#include <assistive_drive/sinyukov_trajectory_generator.h>
#include <ros/ros.h>
#include <base_local_planner/velocity_iterator.h>
namespace assistive_drive_planner {
    
    SinyukovTrajectoryGenerator::SinyukovTrajectoryGenerator(
        double sim_time,
        double sim_granularity,
        double angular_sim_granularity,
        bool use_dwa,
        double sim_period,
        base_local_planner::LocalPlannerLimits* limits,
        Eigen::Vector3f* vsamples,
        bool discretize_by_time,
        bool continued_acceleration
    ) :
        base_local_planner::SimpleTrajectoryGenerator()
    {
        this->setParameters(sim_time, sim_granularity, 
            angular_sim_granularity, use_dwa, sim_period);
        limits_ = limits;
        vsamples_ = vsamples;
        discretize_by_time_ = discretize_by_time;
        continued_acceleration_ = continued_acceleration;
    }


    
    void SinyukovTrajectoryGenerator::initialise(
        const Eigen::Vector3f& pos,
        const Eigen::Vector3f& vel
    ) {
        Eigen::Vector3f goal(100000.0, 100000.0, 0.0); //TODO
        initialise(pos, vel, goal, limits_, *vsamples_, discretize_by_time_);
    }
    
    void SinyukovTrajectoryGenerator::initialise(
        const Eigen::Vector3f& pos,
        const Eigen::Vector3f& vel,
        const Eigen::Vector3f& goal,
        base_local_planner::LocalPlannerLimits* limits,
        const Eigen::Vector3f& vsamples,
        bool discretize_by_time) {
    /*
    * We actually generate all velocity sample vectors here, from which to generate trajectories later on
    */
    double max_vel_th = limits->max_rot_vel;
    double min_vel_th = -1.0 * max_vel_th;
    discretize_by_time_ = discretize_by_time;
    Eigen::Vector3f acc_lim = limits->getAccLimits();
    pos_ = pos;
    vel_ = vel;
    limits_ = limits;
    next_sample_index_ = 0;
    sample_params_.clear();

    double min_vel_x = limits->min_vel_x;
    double max_vel_x = limits->max_vel_x;
    double min_vel_y = limits->min_vel_y;
    double max_vel_y = limits->max_vel_y;
    //ROS_WARN("min_vel_x, max_vel_x %f, %f",min_vel_x, max_vel_x);
    //ROS_WARN(" sample %f, %f, %f",vsamples[0], vsamples[1], vsamples[2]);
    // if sampling number is zero in any dimension, we don't generate samples generically
    if (vsamples[0] * vsamples[1] * vsamples[2] > 0) {
        //compute the feasible velocity space based on the rate at which we run
        Eigen::Vector3f max_vel = Eigen::Vector3f::Zero();
        Eigen::Vector3f min_vel = Eigen::Vector3f::Zero();

        if ( ! use_dwa_) {
            // there is no point in overshooting the goal, and it also may break the
            // robot behavior, so we limit the velocities to those that do not overshoot in sim_time
            double dist =
                sqrt((goal[0] - pos[0]) * (goal[0] - pos[0])) +
                    (goal[1] - pos[1]) * (goal[1] - pos[1]);
            // Factor 2 seems to be necessary, probably a bug somewhere else that I cannot find
            max_vel_x = std::max(std::min(max_vel_x, 2 * dist / sim_time_), min_vel_x);
            max_vel_y = std::max(std::min(max_vel_y, 2 * dist / sim_time_), min_vel_y);

            // if we use continous acceleration, we can sample the max velocity we can reach in sim_time_
            max_vel[0] = std::min(max_vel_x, vel[0] + acc_lim[0] * sim_time_);
            max_vel[1] = std::min(max_vel_y, vel[1] + acc_lim[1] * sim_time_);
            max_vel[2] = std::min(max_vel_th, vel[2] + acc_lim[2] * sim_time_);

            min_vel[0] = std::max(min_vel_x, vel[0] - acc_lim[0] * sim_time_);
            min_vel[1] = std::max(min_vel_y, vel[1] - acc_lim[1] * sim_time_);
            min_vel[2] = std::max(min_vel_th, vel[2] - acc_lim[2] * sim_time_);
        } else {
        // with dwa do not accelerate beyond the first step, we only sample within velocities we reach in sim_period
            //ROS_WARN("vx0, ax, t %f %f, %f",vel[0], acc_lim[0], sim_time_);
            max_vel[0] = std::min(max_vel_x, vel[0] + acc_lim[0] * sim_time_);
            max_vel[1] = std::min(max_vel_y, vel[1] + acc_lim[1] * sim_time_);
            max_vel[2] = std::min(max_vel_th, vel[2] + acc_lim[2] * sim_time_);

            min_vel[0] = std::max(limits->min_vel_x, vel[0] - acc_lim[0] * sim_time_);
            min_vel[1] = std::max(limits->min_vel_y, vel[1] - acc_lim[1] * sim_time_);
            min_vel[2] = std::max(min_vel_th, vel[2] - acc_lim[2] * sim_time_);
        }

        //ROS_WARN("max vel, sample %f, %f",max_vel[0], vsamples[0]);
        Eigen::Vector3f vel = Eigen::Vector3f::Zero();
        for (double vx=min_vel[0]; vx<=max_vel[0]; vx+=max_vel[0]/vsamples[0])
            for (double vy=min_vel[1]; vy<=max_vel[1]; vy+=1)//max_vel[1]/vsamples[1])
                for (double vth=-max_vel[2]; vth<=max_vel[2]; vth+=2*max_vel[2]/vsamples[2])
                    if (fabs(vx)<=limits->max_trans_vel and 
                        (fabs(vx)>=limits->min_trans_vel or fabs(vth)>=min_vel[2])) {
                        //Eigen::Vector3f vel(vx, vy, vth);
                        vel[0]=vx; vel[1]=vy; vel[2]=vth;
                        sample_params_.push_back(vel);
                       // ROS_WARN("Sample %f, %f, %f", vel[0], vel[1], vel[2]);
                    }
        //ROS_WARN("done");
        /*Eigen::Vector3f vel_samp = Eigen::Vector3f::Zero();
        base_local_planner::VelocityIterator x_it(min_vel[0], max_vel[0], vsamples[0]);
        base_local_planner::VelocityIterator y_it(min_vel[1], max_vel[1], vsamples[1]);
        base_local_planner::VelocityIterator th_it(min_vel[2], max_vel[2], vsamples[2]);
        for(; !x_it.isFinished(); x_it++) {
            vel_samp[0] = x_it.getVelocity();
            for(; !y_it.isFinished(); y_it++) {
                vel_samp[1] = y_it.getVelocity();
                for(; !th_it.isFinished(); th_it++) {
                vel_samp[2] = th_it.getVelocity();
                //ROS_DEBUG("Sample %f, %f, %f", vel_samp[0], vel_samp[1], vel_samp[2]);
                sample_params_.push_back(vel_samp);
                }
                th_it.reset();
            }
            y_it.reset();
        }*/
    }
}

};
