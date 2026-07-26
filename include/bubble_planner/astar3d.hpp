#pragma once

#include "bubble_planner/obstacle_index.hpp"
#include "bubble_planner/types.hpp"

#include <Eigen/Core>
#include <memory>
#include <vector>

namespace bubble_planner {

class AStar3D {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AStar3D(std::shared_ptr<const ObstacleIndex> obstacles, AStarConfig config);

  bool search(const Eigen::Vector3d& start,
              const Eigen::Vector3d& goal,
              std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& path,
              std::string* failure_reason = nullptr) const;

  bool lineIsSafe(const Eigen::Vector3d& from,
                  const Eigen::Vector3d& to) const;

 private:
  std::shared_ptr<const ObstacleIndex> obstacles_;
  AStarConfig config_;
};

}  // namespace bubble_planner
