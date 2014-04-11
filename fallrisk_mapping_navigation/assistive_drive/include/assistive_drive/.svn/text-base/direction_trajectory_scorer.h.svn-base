#ifndef ASSISTIVE_DRIVE_DIRECTION_TRAJECTORY_SCORER_H_
#define ASSISTIVE_DRIVE_DIRECTION_TRAJECTORY_SCORER_H_

#include <assistive_drive/sinyukov_trajectory_scorer.h>

namespace assistive_drive_planner {

    class DirectionTrajectoryScorer : public SinyukovTrajectoryScorer  {
        public:
            
            DirectionTrajectoryScorer(
                costmap_2d::Costmap2DROS* costmap_ros,  
                double obstacle_scale = 1.0,
                double angular_scale_ = 1.0 
            );
            double scoreTrajectory(base_local_planner::Trajectory& traj);
    };       
            
};

#endif
