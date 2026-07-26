#include "bubble_planner/corridor_generator.hpp"

#include "bubble_planner/geometry.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

namespace bubble_planner {

CorridorGenerator::CorridorGenerator(std::shared_ptr<const ObstacleIndex> obstacles,
                                     CorridorConfig config)
    : obstacles_(std::move(obstacles)),
      config_(std::move(config)),
      random_engine_(static_cast<std::mt19937::result_type>(config_.random_seed)) {
  if (config_.paper_mode) {
    config_.rho_progress = 0.0;
    config_.rho_guide_cost = 0.0;
    config_.adaptive_covariance = false;
  }
}

bool CorridorGenerator::generateOneSphere(const Eigen::Vector3d& center,
                                          Sphere& sphere) const {
  if (!obstacles_) {
    return false;
  }
  double obstacle_distance = std::numeric_limits<double>::infinity();
  obstacles_->nearest(center, obstacle_distance, nullptr);
  const double clearance = config_.drone_radius + config_.safety_margin;
  double radius = std::isfinite(obstacle_distance)
                      ? obstacle_distance - clearance
                      : config_.max_radius;
  radius = std::min(radius, config_.max_radius);
  sphere.center = center;
  sphere.radius = radius;
  sphere.score = -std::numeric_limits<double>::infinity();
  sphere.reused = false;
  sphere.source_index = std::numeric_limits<std::size_t>::max();
  return radius >= config_.min_radius;
}

Eigen::Vector3d CorridorGenerator::getForwardGuidePoint(
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& path,
    const Sphere& current,
    std::size_t& cursor) const {
  if (path.empty()) {
    return current.center;
  }
  if (path.size() == 1) {
    cursor = 0;
    return path.front();
  }

  // The A* path may have been aggressively simplified.  Looking only at its
  // vertices can therefore place the Gaussian mean several metres beyond the
  // current sphere, where almost no sample can overlap it.  Find the closest
  // point on the remaining polyline and walk forward on the segments instead.
  const std::size_t begin = std::min(cursor, path.size() - 2);
  std::size_t closest_segment = begin;
  double closest_ratio = 0.0;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = begin; i + 1 < path.size(); ++i) {
    const Eigen::Vector3d segment = path[i + 1] - path[i];
    const double squared_length = segment.squaredNorm();
    const double ratio = squared_length > 1.0e-12
        ? clamp((current.center - path[i]).dot(segment) / squared_length, 0.0, 1.0)
        : 0.0;
    const Eigen::Vector3d projected = path[i] + ratio * segment;
    const double distance = (projected - current.center).squaredNorm();
    if (distance < closest_distance) {
      closest_segment = i;
      closest_ratio = ratio;
      closest_distance = distance;
    }
  }

  const double sample_step = std::max(0.05, std::min(0.20, 0.5 * config_.min_radius));
  for (std::size_t i = closest_segment; i + 1 < path.size(); ++i) {
    const Eigen::Vector3d segment_start =
        i == closest_segment
            ? path[i] + closest_ratio * (path[i + 1] - path[i])
            : path[i];
    const Eigen::Vector3d segment = path[i + 1] - segment_start;
    const double length = segment.norm();
    const int samples = std::max(1, static_cast<int>(std::ceil(length / sample_step)));
    for (int sample = 0; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
      const Eigen::Vector3d point = segment_start + ratio * segment;
      if (!current.contains(point, config_.guide_inside_margin)) {
        cursor = i;
        return point;
      }
    }
  }
  cursor = path.size() - 1;
  return path.back();
}

bool CorridorGenerator::batchSample(const Sphere& previous,
                                    const Eigen::Vector3d& guide_point,
                                    const Eigen::Vector3d& goal,
                                    Sphere& selected) {
  Eigen::Vector3d longitudinal = guide_point - previous.center;
  double guide_distance = longitudinal.norm();
  if (guide_distance < 1.0e-6) {
    longitudinal = goal - previous.center;
    guide_distance = longitudinal.norm();
  }
  if (guide_distance < 1.0e-6) {
    return false;
  }
  longitudinal.normalize();

  const Eigen::Vector3d reference = std::abs(longitudinal.z()) < 0.9
                                         ? Eigen::Vector3d::UnitZ()
                                         : Eigen::Vector3d::UnitY();
  Eigen::Vector3d lateral = reference.cross(longitudinal).normalized();
  Eigen::Vector3d vertical = longitudinal.cross(lateral).normalized();

  double sigma_long = config_.longitudinal_sigma_scale * guide_distance;
  double sigma_lateral = config_.lateral_sigma_ratio * sigma_long;
  double sigma_vertical = sigma_lateral;
  if (config_.adaptive_covariance) {
    const double openness = clamp(previous.radius / std::max(config_.min_radius, 1.0e-3),
                                  0.7, 2.0);
    sigma_long *= clamp(0.75 + 0.25 * openness, 0.7, 1.5);
    const double exploration = clamp(1.5 - recent_success_ratio_, 0.7, 1.5);
    sigma_lateral *= exploration;
    sigma_vertical *= exploration;
  }
  sigma_long = std::max(0.05, sigma_long);
  sigma_lateral = std::max(0.03, sigma_lateral);
  sigma_vertical = std::max(0.03, sigma_vertical);

  std::normal_distribution<double> normal(0.0, 1.0);
  bool found = false;
  int accepted = 0;
  Sphere best;
  double best_score = -std::numeric_limits<double>::infinity();
  for (int sample = 0; sample < config_.batch_size; ++sample) {
    const Eigen::Vector3d center = guide_point +
        sigma_long * normal(random_engine_) * longitudinal +
        sigma_lateral * normal(random_engine_) * lateral +
        sigma_vertical * normal(random_engine_) * vertical;

    const double progress = (center - previous.center).dot(longitudinal);
    if (progress < config_.min_forward_progress) {
      continue;
    }

    Sphere candidate;
    if (!generateOneSphere(center, candidate)) {
      continue;
    }
    const double center_distance = (candidate.center - previous.center).norm();
    if (center_distance > previous.radius + candidate.radius - config_.overlap_depth) {
      continue;
    }
    const double overlap = sphereOverlapVolume(previous, candidate);
    if (overlap < config_.min_overlap_volume) {
      continue;
    }

    const double guide_cost = (candidate.center - guide_point).norm();
    const double score = config_.rho_sphere_volume * sphereVolume(candidate.radius) +
                         config_.rho_overlap_volume * overlap +
                         config_.rho_progress * progress -
                         config_.rho_guide_cost * guide_cost;
    candidate.score = score;
    ++accepted;
    if (score > best_score) {
      best_score = score;
      best = candidate;
      found = true;
    }
  }
  recent_success_ratio_ = 0.8 * recent_success_ratio_ +
      0.2 * static_cast<double>(accepted) / std::max(1, config_.batch_size);
  if (found) {
    selected = best;
  }
  return found;
}

bool CorridorGenerator::extend(
    const Eigen::Vector3d& goal,
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& guide_path,
    std::vector<Sphere, Eigen::aligned_allocator<Sphere>>& spheres,
    std::string* failure_reason) {
  if (spheres.empty()) {
    if (failure_reason) *failure_reason = "Cannot extend an empty corridor.";
    return false;
  }
  std::size_t guide_cursor = 0;
  int failed_batches = 0;
  while (!spheres.back().contains(goal) &&
         static_cast<int>(spheres.size()) < config_.max_spheres) {
    const Eigen::Vector3d guide_point =
        getForwardGuidePoint(guide_path, spheres.back(), guide_cursor);
    Sphere next;
    if (!batchSample(spheres.back(), guide_point, goal, next)) {
      ++failed_batches;
      if (failed_batches >= config_.max_failed_batches) {
        if (failure_reason) {
          std::ostringstream stream;
          stream << "Gaussian batch sampling repeatedly failed"
                 << " (spheres=" << spheres.size()
                 << ", remaining=" << (goal - spheres.back().center).norm()
                 << " m, last_radius=" << spheres.back().radius << " m).";
          *failure_reason = stream.str();
        }
        return false;
      }
      continue;
    }
    failed_batches = 0;
    const double center_separation = (next.center - spheres.back().center).norm();
    if (center_separation < 0.05) {
      if (failure_reason) *failure_reason = "Corridor expansion stalled.";
      return false;
    }
    spheres.push_back(next);
  }
  if (!spheres.back().contains(goal)) {
    if (failure_reason) {
      std::ostringstream stream;
      stream << "Corridor reached the maximum number of spheres"
             << " (spheres=" << spheres.size()
             << ", remaining=" << (goal - spheres.back().center).norm()
             << " m, last_radius=" << spheres.back().radius << " m).";
      *failure_reason = stream.str();
    }
    return false;
  }
  return true;
}

void CorridorGenerator::initializeWaypointsAndTimes(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal,
    CorridorResult& result) const {
  result.initial_waypoints.clear();
  result.initial_times.clear();
  if (result.spheres.empty()) {
    return;
  }
  result.initial_waypoints.reserve(result.spheres.size() - 1);
  for (std::size_t i = 0; i + 1 < result.spheres.size(); ++i) {
    result.initial_waypoints.push_back(overlapCenter(result.spheres[i], result.spheres[i + 1]));
  }

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> knots;
  knots.reserve(result.initial_waypoints.size() + 2);
  knots.push_back(start);
  knots.insert(knots.end(), result.initial_waypoints.begin(), result.initial_waypoints.end());
  knots.push_back(goal);
  result.initial_times.reserve(result.spheres.size());
  for (std::size_t i = 0; i + 1 < knots.size(); ++i) {
    const double duration = (knots[i + 1] - knots[i]).norm() /
                            std::max(config_.nominal_speed, 0.1);
    result.initial_times.push_back(std::max(config_.min_piece_time, duration));
  }
}

bool CorridorGenerator::generate(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal,
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& guide_path,
    CorridorResult& result,
    std::string* failure_reason) {
  result = CorridorResult{};
  result.guide_path = guide_path;
  Sphere first;
  if (!generateOneSphere(start, first)) {
    if (failure_reason) {
      double obstacle_distance = std::numeric_limits<double>::infinity();
      obstacles_->nearest(start, obstacle_distance, nullptr);
      const double vehicle_clearance = config_.drone_radius + config_.safety_margin;
      std::ostringstream stream;
      stream << "The start sphere is smaller than min_radius"
             << " (nearest_obstacle=" << obstacle_distance
             << " m, vehicle_clearance=" << vehicle_clearance
             << " m, available_sphere_radius="
             << obstacle_distance - vehicle_clearance
             << " m, required_sphere_radius=" << config_.min_radius << " m).";
      *failure_reason = stream.str();
    }
    return false;
  }
  result.spheres.push_back(first);
  if (!extend(goal, guide_path, result.spheres, failure_reason)) {
    return false;
  }
  initializeWaypointsAndTimes(start, goal, result);
  return true;
}

}  // namespace bubble_planner
