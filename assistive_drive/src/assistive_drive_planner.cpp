#include <assistive_drive/assistive_drive_planner.h>
#include <nav_msgs/Path.h>

namespace assistive_drive_planner {

    AssistiveDrivePlanner::AssistiveDrivePlanner(
        costmap_2d::Costmap2DROS* costmap_ros,
        SinyukovTrajectoryGenerator& traj_generator,
        SinyukovTrajectoryScorer& traj_scorer
    ) :
        generator_(traj_generator),
        scorer_(traj_scorer),
        odom_helper_("odom") 
    {
        costmap_ros_ = costmap_ros;
        //ROS_WARN("Inscribed radius=%.4f", costmap_ros->getInscribedRadius()); //sd1074
        
        ros::NodeHandle nh("~/");
        best_traj_pub_ = nh.advertise<nav_msgs::Path>("best_trajectory", 1, false);
        //try_traj_pub_ = nh.advertise<nav_msgs::Path>("try_trajectory", 1, false);
    }    
    
    double AssistiveDrivePlanner::findBestTrajectory(
         base_local_planner::Trajectory& best_traj
    ) {
        Eigen::Vector3f current_pose = getCurrentPoseAsVector3();
        Eigen::Vector3f current_vel = getCurrentVelocityAsVector3();
        generator_.initialise(current_pose, current_vel);
        scorer_.initialise();
        
        base_local_planner::Trajectory traj;
        double cost, min_cost=-1;
        double max_xv=0;
        //ROS_WARN("!!!!!!!!!!!!!!!!!!!!eeeeeeeeeeeeee");
        while (generator_.hasMoreTrajectories()) {
            if (generator_.nextTrajectory(traj)) {
                //publishTrajectoryAsPath(try_traj_pub_, traj);
                //ROS_WARN("!!!!!!!!!!!!!!!!!!!!");
                cost = scorer_.scoreTrajectory(traj);
                if (traj.xv_>max_xv) {
                    max_xv = traj.xv_;
                }
                //ROS_WARN_NAMED("assistive_drive_planner", "Trajectory seeding velocities: [ %.4f, %.4f, %.4f]. Cost= %.4f\n", traj.xv_, traj.yv_, traj.thetav_, cost);
                //ros::Duration(0.35).sleep();
                if (cost>=0 and (min_cost<0 or cost < min_cost)) {
                    min_cost = cost;
                    best_traj = traj;
                }
            } else {
                //ROS_WARN_NAMED("assistive_drive_planner", "Failed to fetch next trjectory");
            }
        }
        if (min_cost>0) {
            publishTrajectoryAsPath(best_traj_pub_, best_traj);
        }
        //ROS_WARN_NAMED("assistive_drive_planner", "The best trajectory score:  %.4f. The velocity is [ %.4f, %.4f, %.4f].", min_cost, best_traj.xv_, best_traj.yv_, best_traj.thetav_);
        //ROS_WARN("Max Vx=%.4f",max_xv);
        return min_cost;
    }

    
   Eigen::Vector3f AssistiveDrivePlanner::getCurrentPoseAsVector3() {
        //Get current robot pose
        tf::Stamped<tf::Pose> current_pose;
        if ( ! costmap_ros_->getRobotPose(current_pose)) {
            ROS_ERROR("Could not get robot pose");
        }
        Eigen::Vector3f current_pose_v3( //current robot pose:  x , y, [hi
            current_pose.getOrigin().getX(), 
            current_pose.getOrigin().getY(), 
            tf::getYaw(current_pose.getRotation()));
        return current_pose_v3;
    }
    
    Eigen::Vector3f AssistiveDrivePlanner::getCurrentVelocityAsVector3() {
        tf::Stamped<tf::Pose> current_vel;
        odom_helper_.getRobotVel(current_vel);
        Eigen::Vector3f current_vel_v3( //current robot velocity: vx, vy, omega
            current_vel.getOrigin().getX(), 
            current_vel.getOrigin().getY(), 
            tf::getYaw(current_vel.getRotation()));
        return current_vel_v3;
    }
    
    void AssistiveDrivePlanner::publishTrajectoryAsPath(ros::Publisher pub, base_local_planner::Trajectory traj) {
        std::vector<geometry_msgs::PoseStamped> path;

        for(unsigned int i = 0; i < traj.getPointsSize(); ++i) {
            double p_x, p_y, p_th;
            traj.getPoint(i, p_x, p_y, p_th);

            tf::Stamped<tf::Pose> p =
                    tf::Stamped<tf::Pose>(tf::Pose(
                            tf::createQuaternionFromYaw(p_th),
                            tf::Point(p_x, p_y, 0.0)),
                            ros::Time::now(),
                            costmap_ros_->getGlobalFrameID());
            geometry_msgs::PoseStamped pose;
            tf::poseStampedTFToMsg(p, pose);
            path.push_back(pose);
        }

        //publish information to the visualizer
        
        nav_msgs::Path msg;
        msg.header.frame_id="map"; //TODO don't hard code
        msg.poses = path;
        pub.publish(msg);
    }

};
