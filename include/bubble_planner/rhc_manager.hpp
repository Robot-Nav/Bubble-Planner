#pragma once

#include "bubble_planner/corridor_generator.hpp"
#include "bubble_planner/obstacle_index.hpp"
#include "bubble_planner/types.hpp"

#include <Eigen/Core>
#include <cstddef>
#include <memory>
#include <vector>

namespace bubble_planner {

struct ReuseSelection {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<Sphere, Eigen::aligned_allocator<Sphere>> spheres;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> hot_waypoints;
  std::vector<double> hot_times;
  std::size_t source_begin{0};
};

class RhcManager {
 public:
  RhcManager(std::shared_ptr<const ObstacleIndex> obstacles,
             CorridorConfig corridor_config,
             RhcConfig rhc_config);

  ReuseSelection select(const PlanResult& previous,
                        const Eigen::Vector3d& current_position) const;

 private:
  std::shared_ptr<const ObstacleIndex> obstacles_;
  CorridorConfig corridor_config_;
  RhcConfig config_;
};

}  // namespace bubble_planner
