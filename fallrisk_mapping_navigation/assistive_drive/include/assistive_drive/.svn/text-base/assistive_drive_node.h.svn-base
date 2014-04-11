#ifndef ASSISTIVE_DRIVE_NODE_H_
#define ASSISTIVE_DRIVE_NODE_H_
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <assistive_drive/assistive_drive_planner.h>
#include <dynamic_reconfigure/server.h>
#include "assistive_drive/AssistiveDriveConfig.h"

class AssistiveDrive {
        public:
            dynamic_reconfigure::Server<assistive_drive::AssistiveDriveConfig>
                *dyn_prm_srv_;
            tf::TransformListener& tf_; 
            assistive_drive_planner::SinyukovTrajectoryGenerator* traj_generator_;
            assistive_drive_planner::SinyukovTrajectoryScorer* traj_scorer_;
            bool stopped_;
            bool direction_mode_;            
            
            ros::Publisher twist_pub_;
            ros::Subscriber twist_sub_;
            ros::Publisher goal_pub_;
            ros::Publisher desired_traj_pub_;
            assistive_drive_planner::AssistiveDrivePlanner *local_planner_;
            

            AssistiveDrive();
            virtual ~AssistiveDrive();

            void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
            void stopRobot();
            void runLocalPlanner();
            void reconfigCallback(assistive_drive::AssistiveDriveConfig &cfg, uint32_t level);
            boost::recursive_mutex reconfig_mutex_;
            ros::Rate* rate_;
            costmap_2d::Costmap2DROS* costmap_;
        private:
            
};
#endif
