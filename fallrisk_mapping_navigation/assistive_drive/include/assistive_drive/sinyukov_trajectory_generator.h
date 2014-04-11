#ifndef ASSISTIVE_DRIVE_SINYUKOV_TRAJECTORY_GENERATOR_H_
#define ASSISTIVE_DRIVE_SINYUKOV_TRAJECTORY_GENERATOR_H_

#include <Eigen/Core>
#include <base_local_planner/simple_trajectory_generator.h>
#include <base_local_planner/local_planner_limits.h>

namespace assistive_drive_planner {

    class SinyukovTrajectoryGenerator : public base_local_planner::SimpleTrajectoryGenerator {
        public:
            SinyukovTrajectoryGenerator(
                double sim_time,
                double sim_granularity,
                double angular_sim_granularity,
                bool use_dwa,
                double sim_period,
                base_local_planner::LocalPlannerLimits* limits,
                Eigen::Vector3f* vsamples,
                bool discretize_by_time,
                bool continued_acceleration
            );
            
            void initialise(
                const Eigen::Vector3f& pos,
                const Eigen::Vector3f& vel
            );
            
            void initialise(
                const Eigen::Vector3f& pos,
                const Eigen::Vector3f& vel,
                const Eigen::Vector3f& goal,
                base_local_planner::LocalPlannerLimits* limits,
                const Eigen::Vector3f& vsamples,
                bool discretize_by_time
            );
            
            Eigen::Vector3f* vsamples_;
    };
};

#endif

