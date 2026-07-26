#include "bubble_planner/rhc_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace bubble_planner {

RhcManager::RhcManager(std::shared_ptr<const ObstacleIndex> obstacles,
                       CorridorConfig corridor_config,
                       RhcConfig rhc_config)
    : obstacles_(std::move(obstacles)),
      corridor_config_(std::move(corridor_config)),
      config_(std::move(rhc_config)) {}

ReuseSelection RhcManager::select(const PlanResult& previous,
                                  const Eigen::Vector3d& current_position) const {
  ReuseSelection selection;
  if (!config_.enabled || !previous.success || previous.corridor.spheres.empty()) {
    return selection;
  }

  std::size_t begin = 0;
  double closest = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < previous.corridor.spheres.size(); ++i) {
    const double distance = (previous.corridor.spheres[i].center - current_position).norm();
    if (distance < closest) {
      begin = i;
      closest = distance;
    }
  }
  selection.source_begin = begin;

  const double clearance = corridor_config_.drone_radius + corridor_config_.safety_margin;
  for (std::size_t i = begin;
       i < previous.corridor.spheres.size() &&
       static_cast<int>(selection.spheres.size()) < config_.max_reused_spheres;
       ++i) {
    Sphere sphere = previous.corridor.spheres[i];
    if ((sphere.center - current_position).norm() > config_.reuse_distance &&
        !selection.spheres.empty()) {
      break;
    }
    double nearest_distance = std::numeric_limits<double>::infinity();
    obstacles_->nearest(sphere.center, nearest_distance, nullptr);
    const double validated_radius = std::isfinite(nearest_distance)
        ? nearest_distance - clearance - config_.radius_revalidation_margin
        : sphere.radius;
    sphere.radius = std::min(sphere.radius, validated_radius);
    if (sphere.radius < corridor_config_.min_radius) {
      break;
    }
    sphere.reused = true;
    sphere.source_index = i;
    selection.spheres.push_back(sphere);
  }

  if (selection.spheres.empty()) {
    return selection;
  }

  const std::size_t waypoint_begin = begin;
  const std::size_t waypoint_count = selection.spheres.size() > 0
      ? selection.spheres.size() - 1
      : 0;
  for (std::size_t i = 0; i < waypoint_count; ++i) {
    const std::size_t source = waypoint_begin + i;
    if (source < previous.optimized_waypoints.size()) {
      selection.hot_waypoints.push_back(previous.optimized_waypoints[source]);
    }
  }
  for (std::size_t i = 0; i < selection.spheres.size(); ++i) {
    const std::size_t source = begin + i;
    if (source < previous.optimized_times.size()) {
      selection.hot_times.push_back(previous.optimized_times[source]);
    }
  }
  return selection;
}

}  // namespace bubble_planner
