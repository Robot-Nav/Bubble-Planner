#pragma once

#include "bubble_planner/obstacle_index.hpp"
#include "bubble_planner/types.hpp"

#include <Eigen/Core>
#include <cstddef>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace bubble_planner {

class CorridorGenerator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CorridorGenerator(std::shared_ptr<const ObstacleIndex> obstacles,
                    CorridorConfig config);

  bool generate(
      const Eigen::Vector3d& start,
      const Eigen::Vector3d& goal,
      const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& guide_path,
      CorridorResult& result,
      std::string* failure_reason = nullptr);

  bool extend(
      const Eigen::Vector3d& goal,
      const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& guide_path,
      std::vector<Sphere, Eigen::aligned_allocator<Sphere>>& spheres,
      std::string* failure_reason = nullptr);

  bool generateOneSphere(const Eigen::Vector3d& center, Sphere& sphere) const;
  void initializeWaypointsAndTimes(
      const Eigen::Vector3d& start,
      const Eigen::Vector3d& goal,
      CorridorResult& result) const;

 private:
  bool batchSample(const Sphere& previous,
                   const Eigen::Vector3d& guide_point,
                   const Eigen::Vector3d& goal,
                   Sphere& selected);

  Eigen::Vector3d getForwardGuidePoint(
      const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& path,
      const Sphere& current,
      std::size_t& cursor) const;

  std::shared_ptr<const ObstacleIndex> obstacles_;
  CorridorConfig config_;
  std::mt19937 random_engine_;
  double recent_success_ratio_{1.0};
};

}  // namespace bubble_planner
