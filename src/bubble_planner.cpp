#include "bubble_planner/bubble_planner.hpp"

#include "bubble_planner/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

namespace bubble_planner {

BubblePlanner::BubblePlanner(std::shared_ptr<ObstacleIndex> obstacles,
                             PlannerConfig config)
    : obstacles_(std::move(obstacles)),
      config_(std::move(config)),
      astar_(obstacles_, config_.astar),
      corridor_generator_(obstacles_, config_.corridor),
      rhc_manager_(obstacles_, config_.corridor, config_.rhc),
      minco_optimizer_(config_.minco) {}

bool BubblePlanner::createCorridor(const BoundaryState& start,
                                   const BoundaryState& goal,
                                   CorridorResult& corridor,
                                   const PlanResult* previous,
                                   std::string* failure_reason) {
  ReuseSelection reuse;
  if (previous != nullptr) {
    reuse = rhc_manager_.select(*previous, start.position);
  }

  if (!reuse.spheres.empty()) {
    corridor = CorridorResult{};
    corridor.spheres = reuse.spheres;
    corridor.reused_sphere_count = reuse.spheres.size();

    if (!corridor.spheres.front().contains(start.position)) {
      Sphere start_sphere;
      if (!corridor_generator_.generateOneSphere(start.position, start_sphere)) {
        corridor.spheres.clear();
      } else if ((start_sphere.center - corridor.spheres.front().center).norm() <=
                 start_sphere.radius + corridor.spheres.front().radius -
                     config_.corridor.overlap_depth) {
        corridor.spheres.insert(corridor.spheres.begin(), start_sphere);
        corridor.reused_sphere_count = reuse.spheres.size();
      } else {
        corridor.spheres.clear();
      }
    }

    if (!corridor.spheres.empty()) {
      std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> guide_path;
      std::string astar_error;
      const Eigen::Vector3d extension_start = corridor.spheres.back().center;
      if (!astar_.search(extension_start, goal.position, guide_path, &astar_error)) {
        if (failure_reason) {
          *failure_reason = "RHC extension A* failed: " + astar_error;
        }
        return false;
      }
      corridor.guide_path = guide_path;
      if (!corridor_generator_.extend(goal.position,
                                      guide_path,
                                      corridor.spheres,
                                      failure_reason)) {
        return false;
      }
      corridor_generator_.initializeWaypointsAndTimes(start.position,
                                                       goal.position,
                                                       corridor);

      const std::size_t hot_waypoint_count = std::min(
          reuse.hot_waypoints.size(),
          corridor.reused_sphere_count > 0 ? corridor.reused_sphere_count - 1 : 0);
      for (std::size_t i = 0; i < hot_waypoint_count; ++i) {
        if (corridor.spheres[i].contains(reuse.hot_waypoints[i]) &&
            corridor.spheres[i + 1].contains(reuse.hot_waypoints[i])) {
          corridor.initial_waypoints[i] = reuse.hot_waypoints[i];
        }
      }
      const std::size_t hot_time_count = std::min(reuse.hot_times.size(),
                                                  corridor.reused_sphere_count);
      for (std::size_t i = 0; i < hot_time_count; ++i) {
        corridor.initial_times[i] = std::max(config_.minco.min_piece_time,
                                             reuse.hot_times[i]);
      }
      return true;
    }
  }

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> guide_path;
  std::string astar_error;
  if (!astar_.search(start.position, goal.position, guide_path, &astar_error)) {
    if (failure_reason) *failure_reason = "A* failed: " + astar_error;
    return false;
  }
  return corridor_generator_.generate(start.position,
                                      goal.position,
                                      guide_path,
                                      corridor,
                                      failure_reason);
}

bool BubblePlanner::trajectoryIsCollisionFree(const Trajectory<7>& trajectory,
                                              double time_horizon,
                                              double sample_dt,
                                              std::string* failure_reason) const {
  if (trajectory.getPieceNum() <= 0) {
    if (failure_reason) *failure_reason = "Trajectory is empty.";
    return false;
  }
  const double clearance = config_.corridor.drone_radius +
                           config_.corridor.safety_margin;
  const double duration = std::min(trajectory.getTotalDuration(), time_horizon);
  const double dt = std::max(0.01, sample_dt);
  for (double t = 0.0; t <= duration + 1.0e-9; t += dt) {
    const Eigen::Vector3d position = trajectory.getPos(std::min(t, duration));
    if (!obstacles_->isSafe(position, clearance)) {
      if (failure_reason) {
        std::ostringstream stream;
        stream << "Obstacle collision at trajectory time " << t << " s.";
        *failure_reason = stream.str();
      }
      return false;
    }
  }
  return true;
}

bool BubblePlanner::plan(const BoundaryState& start,
                         const BoundaryState& goal,
                         PlanResult& result,
                         const PlanResult* previous) {
  result = PlanResult{};
  result.start_state = start;
  result.goal_state = goal;
  if (!obstacles_ || !obstacles_->ready()) {
    result.message = "Obstacle index is not ready.";
    return false;
  }
  double start_distance = std::numeric_limits<double>::infinity();
  double goal_distance = std::numeric_limits<double>::infinity();
  obstacles_->nearest(start.position, start_distance, nullptr);
  obstacles_->nearest(goal.position, goal_distance, nullptr);
  if (start_distance < config_.astar.clearance ||
      goal_distance < config_.astar.clearance) {
    std::ostringstream stream;
    stream << "Start or local goal is inside the clearance envelope"
           << " (start_nearest=" << start_distance
           << " m, goal_nearest=" << goal_distance
           << " m, required=" << config_.astar.clearance << " m).";
    result.message = stream.str();
    return false;
  }

  std::string corridor_error;
  if (!createCorridor(start, goal, result.corridor, previous, &corridor_error)) {
    result.message = corridor_error;
    return false;
  }

  std::string optimization_error;
  if (!minco_optimizer_.optimize(start,
                                 goal,
                                 result.corridor.spheres,
                                 result.corridor.initial_waypoints,
                                 result.corridor.initial_times,
                                 result.trajectory,
                                 result.optimized_waypoints,
                                 result.optimized_times,
                                 &optimization_error)) {
    result.message = optimization_error;
    return false;
  }

  std::string collision_error;
  if (!trajectoryIsCollisionFree(result.trajectory,
                                 result.trajectory.getTotalDuration(),
                                 std::min(0.03, config_.collision_check_dt),
                                 &collision_error)) {
    result.message = "Independent collision check failed: " + collision_error;
    return false;
  }
  result.success = true;
  result.message = "Planning succeeded.";
  return true;
}

}  // namespace bubble_planner
