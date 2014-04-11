#ifndef ASSISTIVE_DRIVE_SINYUKOV_TRAJECTORY_SCORER_H_
#define ASSISTIVE_DRIVE_SINYUKOV_TRAJECTORY_SCORER_H_

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/obstacle_cost_function.h>

namespace assistive_drive_planner {

    class SinyukovTrajectoryScorer {
        public:
            base_local_planner::ObstacleCostFunction* obstacle_scorer_;
            Eigen::Vector3f desired_;
            costmap_2d::Costmap2DROS* costmap_ros_;
            costmap_2d::Costmap2D* costmap_;            
            double obstacle_scale_;
            double angular_scale_;
            
            
            SinyukovTrajectoryScorer(
                costmap_2d::Costmap2DROS* costmap_ros,  
                double obstacle_scale = 1.0,
                double angular_scale_ = 1.0 
            );
            
            void initialise();
            
            void setDesired(double x, double y, double phi);
    
            virtual double scoreTrajectory(base_local_planner::Trajectory& traj);
    };       
            
};

#endif
