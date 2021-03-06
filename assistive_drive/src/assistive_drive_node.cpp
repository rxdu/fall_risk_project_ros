/*********************************************************************
* Author: Dmitry Sinyukov
*********************************************************************/
#include <assistive_drive/assistive_drive_node.h>
#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/Path.h>
#include <assistive_drive/direction_trajectory_scorer.h>

AssistiveDrive::AssistiveDrive() :
    tf_(*new tf::TransformListener(ros::Duration(10)))
    {
    ros::NodeHandle n;

// Create a new costmap to store local_costmap --look into the implementation of costmap later.
    costmap_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    //ROS_WARN("11");
    rate_ = NULL;
    //ROS_WARN("22");

// Set the values of parameters from the cfg file using dynamic_reconfugre package. This allows user to change parameters without restarting the node
    dyn_prm_srv_ = new dynamic_reconfigure::Server
        <assistive_drive::AssistiveDriveConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server
        <assistive_drive::AssistiveDriveConfig>::CallbackType 
        dyn_prm_callback = boost::bind(&AssistiveDrive::reconfigCallback, this, _1, _2);
    dyn_prm_srv_->setCallback(dyn_prm_callback);

//	???? What sorcery is this?
    while(rate_ == NULL) {};

//    Call twistCallBack function -- assuming that /rough_cmd_vel is sending the commands to move
    twist_sub_ = n.subscribe<geometry_msgs::Twist>(
        "/rough_cmd_vel", 
        2,
        boost::bind(&AssistiveDrive::twistCallback, this, _1));

    twist_pub_ = n.advertise<geometry_msgs::Twist>(
        "/cmd_vel", 
        2);
    goal_pub_ = n.advertise<geometry_msgs::PoseStamped>(
        "/goal", 
        2);    
    new boost::thread(boost::bind(&AssistiveDrive::runLocalPlanner, this));
}


void AssistiveDrive::reconfigCallback(
        assistive_drive::AssistiveDriveConfig &cfg, uint32_t level) {
    //boost::recursive_mutex::scoped_lock l(reconfig_mutex_);

    rate_ =  new ros::Rate(cfg.rate);

    //Trajectory Generator parameters
    double sim_time, sim_granularity, angular_sim_granularity;
    bool use_dwa, discretize_by_time, continued_acceleration;
    base_local_planner::LocalPlannerLimits* limits = 
        new base_local_planner::LocalPlannerLimits(); //read generator limits
    
    sim_time                = cfg.gen_sim_time;
    sim_granularity         = cfg.gen_sim_granularity;
    angular_sim_granularity = cfg.gen_angular_sim_granularity;
    use_dwa                 = cfg.gen_use_dwa;
    discretize_by_time      = cfg.gen_discretize_by_time;
    continued_acceleration  = cfg.gen_continued_acceleration;
    limits->max_trans_vel   = cfg.gen_limits_max_trans_vel;
    limits->min_trans_vel   = cfg.gen_limits_min_trans_vel;
    limits->max_vel_x       = cfg.gen_limits_max_vel_x;
    limits->min_vel_x       = cfg.gen_limits_min_vel_x;
    limits->max_vel_y       = cfg.gen_limits_max_vel_y;
    limits->min_vel_y       = cfg.gen_limits_min_vel_y;
    limits->max_rot_vel     = cfg.gen_limits_max_rot_vel;
    limits->min_rot_vel     = cfg.gen_limits_min_rot_vel;
    limits->acc_limit_trans = cfg.gen_limits_acc_limit_trans;
    limits->acc_lim_x       = cfg.gen_limits_acc_lim_x;
    limits->acc_lim_y       = cfg.gen_limits_acc_lim_y;
    limits->acc_lim_theta   = cfg.gen_limits_acc_lim_theta;
    int samples_vx, samples_vy, samples_vth;
    samples_vx = cfg.gen_samples_vx;
    samples_vy = cfg.gen_samples_vy;
    samples_vth = cfg.gen_samples_vth;
    Eigen::Vector3f* vsamples = new Eigen::Vector3f(samples_vx, samples_vy, samples_vth);
    //Eigen::Vector3f vsamples(samples_vx, samples_vy, samples_vth);
    //Trajectory Scorer parameters
    double obstacle_scale, angular_scale;
    direction_mode_ = cfg.scorer_direction_mode;
    obstacle_scale  = cfg.scorer_obstacle_scale;
    angular_scale   = cfg.scorer_angular_scale;

    
    //Create Generator, Scorer => Planner
    traj_generator_ = new assistive_drive_planner::SinyukovTrajectoryGenerator(
        sim_time, sim_granularity, angular_sim_granularity, use_dwa, 1/cfg.rate,
        limits, vsamples, discretize_by_time, continued_acceleration
    );
    
    traj_scorer_ = (direction_mode_) ? 
        new assistive_drive_planner::DirectionTrajectoryScorer(
            costmap_, obstacle_scale, angular_scale):
        new assistive_drive_planner::SinyukovTrajectoryScorer(
            costmap_, obstacle_scale, angular_scale);    
    
    local_planner_ = new assistive_drive_planner::AssistiveDrivePlanner(
        costmap_, *traj_generator_, *traj_scorer_);
    
    ROS_DEBUG("assistive_drive node reconfigured");
}

void AssistiveDrive::runLocalPlanner() {
    ROS_WARN("Direction mode is %s", (direction_mode_) ? "ON": "OFF");

    ros::NodeHandle nh("~");
    stopped_ = true;
    
    desired_traj_pub_ = nh.advertise<nav_msgs::Path>("desired_trajectory", 1, false);
    
    geometry_msgs::Twist cmd_vel; //to be updated with the velocities from local planner
    while(nh.ok()) {
        //boost::recursive_mutex::scoped_lock ecl(reconfig_mutex_);
        //for timing that gives real time even in simulation
        ros::WallTime start = ros::WallTime::now();

        //check that the observation buffers for the costmap are current, we don't want to drive blind
        if(!costmap_->isCurrent()){
            ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
            stopRobot();
        } else if(!stopped_) {
            base_local_planner::Trajectory best_trajectory;
            double cost = local_planner_->findBestTrajectory(best_trajectory);
            if (cost<0) {
                ROS_WARN("Can't find a valid trajectory. Keep trying.");
                //stopRobot();
            } else {
                cmd_vel.linear.x = best_trajectory.xv_;
                cmd_vel.linear.y = best_trajectory.yv_;
                cmd_vel.angular.z = best_trajectory.thetav_;

                ROS_DEBUG_NAMED("assistive_drive", "Got a valid command from the local planner. Best tejctoy cost: %.4f", cost);
                twist_pub_.publish(cmd_vel);
                //ROS_WARN("Best trajectory cost %.4f, seeding velocities: [%.4f, %.4f, %.4f].\n", cost, best_trajectory.xv_, best_trajectory.yv_, best_trajectory.thetav_);
                //local_planner_->publishTrajectoryAsPath(best_trajectory);
            }
        }
        ros::WallDuration t_diff = ros::WallTime::now() - start;
        ROS_DEBUG_NAMED("assistive_drive","Full control cycle time: %.9f\n", t_diff.toSec());
        double w = rate_->expectedCycleTime().toSec();
        //ROS_WARN("RUN BEFORE SLEEP T=%f", w);
        rate_->sleep(); //make sure to sleep for the remainder of our cycle time
        if(rate_->cycleTime().toSec() > w ) {
            ROS_WARN("Local planner missed its desired rate of %.4fHz (period=%.4f sec)... the loop actually took %.4f seconds", 
                1/w, w, rate_->cycleTime().toSec());
        }
    }
}

// Stop the robot -- why is the variable not initialized? assumption: Constructor initializes it to 0
void AssistiveDrive::stopRobot() {
    geometry_msgs::Twist cmd_vel;
    twist_pub_.publish(cmd_vel);
    stopped_ = true;
}



void AssistiveDrive::twistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    double vx=msg->linear.x, vy=msg->linear.y, omega=msg->angular.z;

// if all the velocities in the msg object are zero, stop the robot, else reset the flag
    if (vx==0.0 and vy==0.0 and omega==0.0) {
        stopRobot();
    } else {
        stopped_=false;
    }
    
// direction_mode_ is set using cfg file -- read about it later
    if (direction_mode_) {

//get the transform between map and base_link
    	tf::StampedTransform transform;
        try {
            tf_.lookupTransform("/map", "/base_link", ros::Time(0), transform);
        } catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

// Calculate velocities and angle for goal
        double c = 2;
        double x1 = c * vx;
        double y1 = c * 0.35 * omega;
        double yaw1 = tf::getYaw(transform.getRotation());
        
        double x_ = x1*cos(yaw1) - y1*sin(yaw1);
        double y_ = x1*sin(yaw1) + y1*cos(yaw1);
        
        double x, y, phi;
        x = transform.getOrigin().x() + x_;
        y = transform.getOrigin().y() + y_;
        phi = atan2(y_,x_);

// Assuming that the line below score the trajectory. set x,y & Phi as the desired goal
        traj_scorer_->setDesired(x, y, phi);
        
        //Visualize directions:
        geometry_msgs::PoseStamped p;
        p.header.frame_id = "/map";
        p.pose.position.x = x;
        p.pose.position.y = y;
        p.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, phi);
        goal_pub_.publish(p);

    }

// If direction mode is not set, the desired x,y,z are same as those of msg.
    else {
        traj_scorer_->setDesired(vx, vy, omega);
        
        //Visualize desired trajectory/velocity:
        base_local_planner::Trajectory desired_traj;
        traj_generator_->generateTrajectory(
            local_planner_->getCurrentPoseAsVector3(), 
            local_planner_->getCurrentVelocityAsVector3(),
            Eigen::Vector3f(vx, vy, omega),
            desired_traj            
        );
        local_planner_->publishTrajectoryAsPath(desired_traj_pub_, desired_traj);
    }
}

AssistiveDrive::~AssistiveDrive() {}

int main(int argc, char **argv) {
    ros::init(argc, argv, "assistive_drive_node");
    AssistiveDrive *d = new AssistiveDrive();
    //ros::MultiThreadedSpinner spinner(4);
    ros::spin();
    return 0;
}
