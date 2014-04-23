#include <cfloat>
#include <assistive_drive/direction_trajectory_scorer.h>
#include <csignal> //TODO

namespace assistive_drive_planner {
    
    DirectionTrajectoryScorer::DirectionTrajectoryScorer(
        costmap_2d::Costmap2DROS* costmap_ros,
        double obstacle_scale,
        double angular_scale
    ) : SinyukovTrajectoryScorer(costmap_ros, obstacle_scale, angular_scale)  {}

   
    double DirectionTrajectoryScorer::scoreTrajectory(base_local_planner::Trajectory& traj) {
        obstacle_scorer_->prepare();
        //ROS_WARN("Scorer After Prepare ROS Inscribed radius=%.4f", costmap_ros_->getInscribedRadius()); //sd1074
        double obstacle_cost = obstacle_scorer_->scoreTrajectory(traj);
        ROS_WARN("Obstacle cost: %f", obstacle_cost);
        if (obstacle_cost < 0)
            return obstacle_cost;
        
        double px, py, pth;
        traj.getPoint(traj.getPointsSize()-1, px, py, pth);
        //ROS_WARN("tx=%f, dx=%f", px, desired_[0]);
        double d2 = (px-desired_[0])*(px-desired_[0]) + (py-desired_[1])*(py-desired_[1]);
        return obstacle_scale_*obstacle_cost + d2;
    }
};
