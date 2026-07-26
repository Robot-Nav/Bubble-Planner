#pragma once

#include "bubble_planner/types.hpp"

#include <Eigen/Core>
#include <gcopter/lbfgs.hpp>
#include <gcopter/minco.hpp>
#include <gcopter/trajectory.hpp>

#include <vector>

namespace bubble_planner {

class MincoOptimizer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit MincoOptimizer(MincoConfig config);

  bool optimize(
      const BoundaryState& start,
      const BoundaryState& goal,
      const std::vector<Sphere, Eigen::aligned_allocator<Sphere>>& corridor,
      const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& initial_waypoints,
      const std::vector<double>& initial_times,
      Trajectory<7>& trajectory,
      std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& optimized_waypoints,
      std::vector<double>& optimized_times,
      std::string* failure_reason = nullptr);

  bool validate(
      const Trajectory<7>& trajectory,
      const std::vector<Sphere, Eigen::aligned_allocator<Sphere>>& corridor,
      std::string* failure_reason = nullptr) const;

 private:
  static double objectiveCallback(void* instance,
                                  const Eigen::VectorXd& variables,
                                  Eigen::VectorXd& gradient);

  double evaluateObjective(const Eigen::VectorXd& variables,
                           Trajectory<7>* trajectory = nullptr) const;
  double evaluateObjectiveAndGradient(const Eigen::VectorXd& variables,
                                      Eigen::VectorXd& gradient) const;
  void decodeVariables(
      const Eigen::VectorXd& variables,
      Eigen::MatrixXd& waypoints,
      Eigen::VectorXd& times) const;

  MincoConfig config_;
  BoundaryState start_;
  BoundaryState goal_;
  std::vector<Sphere, Eigen::aligned_allocator<Sphere>> corridor_;
  int piece_count_{0};
  mutable minco::MINCO_S4NU minco_;
};

}  // namespace bubble_planner
