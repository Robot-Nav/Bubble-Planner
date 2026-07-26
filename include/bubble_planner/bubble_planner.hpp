#pragma once

#include "bubble_planner/astar3d.hpp"
#include "bubble_planner/corridor_generator.hpp"
#include "bubble_planner/minco_optimizer.hpp"
#include "bubble_planner/obstacle_index.hpp"
#include "bubble_planner/rhc_manager.hpp"
#include "bubble_planner/types.hpp"

#include <Eigen/Core>
#include <memory>
#include <string>

namespace bubble_planner {

class BubblePlanner {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BubblePlanner(std::shared_ptr<ObstacleIndex> obstacles, PlannerConfig config);

  bool plan(const BoundaryState& start,
            const BoundaryState& goal,
            PlanResult& result,
            const PlanResult* previous = nullptr);

  bool trajectoryIsCollisionFree(const Trajectory<7>& trajectory,
                                 double time_horizon,
                                 double sample_dt,
                                 std::string* failure_reason = nullptr) const;

 private:
  bool createCorridor(const BoundaryState& start,
                      const BoundaryState& goal,
                      CorridorResult& corridor,
                      const PlanResult* previous,
                      std::string* failure_reason);

  std::shared_ptr<ObstacleIndex> obstacles_;
  PlannerConfig config_;
  AStar3D astar_;
  CorridorGenerator corridor_generator_;
  RhcManager rhc_manager_;
  MincoOptimizer minco_optimizer_;
};

}  // namespace bubble_planner
