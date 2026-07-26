#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <gcopter/trajectory.hpp>

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace bubble_planner {

struct Sphere {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double radius{0.0};
  double score{-std::numeric_limits<double>::infinity()};
  bool reused{false};
  std::size_t source_index{std::numeric_limits<std::size_t>::max()};

  [[nodiscard]] bool contains(const Eigen::Vector3d& p,
                              double tolerance = 0.0) const {
    const double effective = radius + tolerance;
    return effective >= 0.0 && (p - center).squaredNorm() <= effective * effective;
  }
};

struct BoundaryState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
};

struct AStarConfig {
  double resolution{0.25};
  double clearance{0.15};
  double search_margin{2.0};
  double min_z{0.2};
  double max_z{5.0};
  double proximity_weight{0.08};
  std::size_t max_expanded_nodes{120000};
  bool simplify_path{true};
};

struct CorridorConfig {
  bool paper_mode{true};
  int random_seed{7};
  int batch_size{36};
  int max_spheres{18};
  int max_failed_batches{4};
  double drone_radius{0.30};
  double safety_margin{0.15};
  double min_radius{0.20};
  double max_radius{4.0};
  double overlap_depth{0.12};
  double min_overlap_volume{0.01};
  double guide_inside_margin{0.05};
  double min_forward_progress{0.05};
  double nominal_speed{3.0};
  double min_piece_time{0.12};
  double rho_sphere_volume{1.0};
  double rho_overlap_volume{2.0};
  double rho_progress{0.0};
  double rho_guide_cost{0.0};
  bool adaptive_covariance{false};
  double longitudinal_sigma_scale{0.50};
  double lateral_sigma_ratio{0.50};
};

struct RhcConfig {
  bool enabled{true};
  double reuse_distance{3.0};
  int max_reused_spheres{5};
  double radius_revalidation_margin{0.02};
};

struct MincoConfig {
  double max_velocity{6.0};
  double max_acceleration{8.0};
  double max_jerk{25.0};
  double nominal_speed{3.0};
  double min_piece_time{0.12};
  double weight_time{20.0};
  double weight_corridor{8000.0};
  double weight_velocity{2000.0};
  double weight_acceleration{2000.0};
  double weight_jerk{50.0};
  double weight_waypoint{5000.0};
  double smoothing_mu{0.02};
  int quadrature_intervals{8};
  int validation_samples_per_piece{40};
  int max_iterations{45};
  int lbfgs_memory{12};
  double relative_cost_tolerance{1.0e-5};
  double gradient_epsilon{1.0e-5};
  double finite_difference_step{2.0e-5};
};

struct PlannerConfig {
  AStarConfig astar;
  CorridorConfig corridor;
  RhcConfig rhc;
  MincoConfig minco;
  double planning_horizon{8.0};
  double distance_trigger_ratio{0.4};
  double collision_check_horizon{2.0};
  double collision_check_dt{0.05};
  double commit_time{0.12};
};

struct CorridorResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<Sphere, Eigen::aligned_allocator<Sphere>> spheres;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> guide_path;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> initial_waypoints;
  std::vector<double> initial_times;
  std::size_t reused_sphere_count{0};
};

struct PlanResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool success{false};
  std::string message;
  CorridorResult corridor;
  Trajectory<7> trajectory;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> optimized_waypoints;
  std::vector<double> optimized_times;
  BoundaryState start_state;
  BoundaryState goal_state;
};

}  // namespace bubble_planner
