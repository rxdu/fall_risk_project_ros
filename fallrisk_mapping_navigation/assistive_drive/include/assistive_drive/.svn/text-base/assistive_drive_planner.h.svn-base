#ifndef ASSISTIVE_DRIVE_PLANNER_H_
#define ASSISTIVE_DRIVE_PLANNER_H_

#include <costmap_2d/costmap_2d.h>

#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/trajectory.h>
#include <assistive_drive/sinyukov_trajectory_generator.h>
#include <assistive_drive/sinyukov_trajectory_scorer.h>

namespace assistive_drive_planner {
    class AssistiveDrivePlanner {
        public:
            costmap_2d::Costmap2DROS* costmap_ros_;            
            SinyukovTrajectoryGenerator& generator_;
            SinyukovTrajectoryScorer& scorer_;        
            base_local_planner::OdometryHelperRos odom_helper_;
            
            ros::Publisher best_traj_pub_;
            //ros::Publisher try_traj_pub_;
            
            AssistiveDrivePlanner(
                costmap_2d::Costmap2DROS* costmap_ros,
                SinyukovTrajectoryGenerator& generator,
                SinyukovTrajectoryScorer& scorer
            );
            
            double findBestTrajectory(
                base_local_planner::Trajectory& best_traj
            );
            

            Eigen::Vector3f getCurrentPoseAsVector3();
            Eigen::Vector3f getCurrentVelocityAsVector3();
            void publishTrajectoryAsPath(ros::Publisher pub, base_local_planner::Trajectory traj);

    };            
};

#endif
