#include <cfloat>
#include <assistive_drive/sinyukov_trajectory_scorer.h>
#include <csignal> //TODO

namespace assistive_drive_planner {
    
    SinyukovTrajectoryScorer::SinyukovTrajectoryScorer(
        costmap_2d::Costmap2DROS* costmap_ros,
        double obstacle_scale,
        double angular_scale
    ) :
        costmap_ros_(costmap_ros)
    {
        costmap_ = costmap_ros_->getCostmap();
        //ROS_WARN("Scorer ROS Inscribed radius=%.4f", costmap_ros_->getInscribedRadius()); //sd1074
        //ROS_WARN("Scorer no ROS Inscribed radius=%.4f", costmap_.getInscribedRadius()); //sd1074
        obstacle_scorer_ = new base_local_planner::ObstacleCostFunction(costmap_);
        obstacle_scale_ = obstacle_scale;
        angular_scale_ = angular_scale;
        obstacle_scorer_->setParams(0, 0, DBL_MAX);
        setDesired(0.0, 0.0, 0.0);
    }


    
    void SinyukovTrajectoryScorer::initialise() {
        obstacle_scorer_->setFootprint(costmap_ros_->getRobotFootprint());
        //costmap_ros_->clearRobotFootprint(); ///???
        costmap_ = costmap_ros_->getCostmap();
    }

    
    void SinyukovTrajectoryScorer::setDesired(double x, double y, double phi) {
        desired_[0] = x;
        desired_[1] = y;
        desired_[2] = angular_scale_*phi;
    }
    
    double SinyukovTrajectoryScorer::scoreTrajectory(base_local_planner::Trajectory& traj) {
        obstacle_scorer_->prepare();
        //ROS_WARN("Scorer After Prepare ROS Inscribed radius=%.4f", costmap_ros_->getInscribedRadius()); //sd1074
        double obstacle_cost = obstacle_scorer_->scoreTrajectory(traj);
        //ROS_WARN("Ooobstacle cost: %f", obstacle_cost);
        if (obstacle_cost < 0)
            return obstacle_cost;
        
        Eigen::Vector3f traj_vel(traj.xv_, traj.yv_, angular_scale_*traj.thetav_);
        traj_vel -= desired_;
        return obstacle_scale_*obstacle_cost + traj_vel.dot(traj_vel);
    }
};
