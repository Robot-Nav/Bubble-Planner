#include "bubble_planner/astar3d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bubble_planner {
namespace {

struct GridIndex {
  int x{0};
  int y{0};
  int z{0};
  bool operator==(const GridIndex& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridHash {
  std::size_t operator()(const GridIndex& index) const noexcept {
    std::size_t seed = 0;
    auto mix = [&seed](int value) {
      seed ^= std::hash<int>{}(value) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    };
    mix(index.x);
    mix(index.y);
    mix(index.z);
    return seed;
  }
};

struct NodeRecord {
  double g{std::numeric_limits<double>::infinity()};
  double f{std::numeric_limits<double>::infinity()};
  GridIndex parent{};
  bool has_parent{false};
  bool closed{false};
};

struct QueueEntry {
  double f{0.0};
  GridIndex index{};
  bool operator<(const QueueEntry& other) const { return f > other.f; }
};

}  // namespace

AStar3D::AStar3D(std::shared_ptr<const ObstacleIndex> obstacles, AStarConfig config)
    : obstacles_(std::move(obstacles)), config_(std::move(config)) {}

bool AStar3D::lineIsSafe(const Eigen::Vector3d& from,
                         const Eigen::Vector3d& to) const {
  const Eigen::Vector3d delta = to - from;
  const double length = delta.norm();
  if (length < 1.0e-9) {
    return obstacles_->isSafe(from, config_.clearance);
  }
  const double sample_step = std::max(0.05, 0.45 * config_.resolution);
  const int sample_count = std::max(1, static_cast<int>(std::ceil(length / sample_step)));
  for (int i = 0; i <= sample_count; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(sample_count);
    const Eigen::Vector3d point = from + ratio * delta;
    if (point.z() < config_.min_z || point.z() > config_.max_z ||
        !obstacles_->isSafe(point, config_.clearance)) {
      return false;
    }
  }
  return true;
}

bool AStar3D::search(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal,
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& path,
    std::string* failure_reason) const {
  path.clear();
  if (!obstacles_) {
    if (failure_reason) *failure_reason = "Obstacle index is null.";
    return false;
  }
  if (!obstacles_->isSafe(start, config_.clearance) ||
      !obstacles_->isSafe(goal, config_.clearance)) {
    if (failure_reason) *failure_reason = "Start or goal violates clearance.";
    return false;
  }
  if (lineIsSafe(start, goal)) {
    path.push_back(start);
    path.push_back(goal);
    return true;
  }

  const Eigen::Vector3d lower = start.cwiseMin(goal) -
      Eigen::Vector3d::Constant(config_.search_margin);
  const Eigen::Vector3d upper = start.cwiseMax(goal) +
      Eigen::Vector3d::Constant(config_.search_margin);
  const Eigen::Vector3d bounded_lower(lower.x(), lower.y(), std::max(lower.z(), config_.min_z));
  const Eigen::Vector3d bounded_upper(upper.x(), upper.y(), std::min(upper.z(), config_.max_z));
  const double resolution = config_.resolution;

  auto to_grid = [&](const Eigen::Vector3d& point) {
    const Eigen::Array3d scaled = (point - bounded_lower).array() / resolution;
    return GridIndex{static_cast<int>(std::llround(scaled.x())),
                     static_cast<int>(std::llround(scaled.y())),
                     static_cast<int>(std::llround(scaled.z()))};
  };
  auto to_world = [&](const GridIndex& index) -> Eigen::Vector3d {
    return bounded_lower + resolution * Eigen::Vector3d(index.x, index.y, index.z);
  };
  auto inside = [&](const Eigen::Vector3d& point) {
    return (point.array() >= bounded_lower.array()).all() &&
           (point.array() <= bounded_upper.array()).all();
  };

  const GridIndex start_index = to_grid(start);
  const GridIndex goal_index = to_grid(goal);
  std::unordered_map<GridIndex, NodeRecord, GridHash> records;
  std::priority_queue<QueueEntry> open;
  NodeRecord& start_record = records[start_index];
  start_record.g = 0.0;
  start_record.f = (start - goal).norm();
  open.push(QueueEntry{start_record.f, start_index});

  std::vector<GridIndex> offsets;
  offsets.reserve(26);
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx != 0 || dy != 0 || dz != 0) {
          offsets.push_back(GridIndex{dx, dy, dz});
        }
      }
    }
  }

  bool found = false;
  GridIndex reached = start_index;
  std::size_t expanded = 0;
  while (!open.empty() && expanded < config_.max_expanded_nodes) {
    const QueueEntry current_entry = open.top();
    open.pop();
    auto record_it = records.find(current_entry.index);
    if (record_it == records.end() || record_it->second.closed ||
        current_entry.f > record_it->second.f + 1.0e-9) {
      continue;
    }
    NodeRecord& current_record = record_it->second;
    current_record.closed = true;
    // Inserting a neighbor into `records` may rehash the unordered_map and
    // invalidate `current_record`.  Keep the only value needed by the
    // expansion loop before any insertion takes place.
    const double current_g = current_record.g;
    ++expanded;
    const Eigen::Vector3d current_world = to_world(current_entry.index);

    if (current_entry.index == goal_index ||
        (current_world - goal).norm() <= 1.5 * resolution) {
      reached = current_entry.index;
      found = true;
      break;
    }

    for (const GridIndex& offset : offsets) {
      const GridIndex next{current_entry.index.x + offset.x,
                           current_entry.index.y + offset.y,
                           current_entry.index.z + offset.z};
      const Eigen::Vector3d next_world = to_world(next);
      if (!inside(next_world) || !obstacles_->isSafe(next_world, config_.clearance)) {
        continue;
      }
      double obstacle_distance = std::numeric_limits<double>::infinity();
      obstacles_->nearest(next_world, obstacle_distance, nullptr);
      const double step = resolution * std::sqrt(static_cast<double>(
          offset.x * offset.x + offset.y * offset.y + offset.z * offset.z));
      const double free_margin = std::max(0.05, obstacle_distance - config_.clearance);
      const double proximity_cost = config_.proximity_weight / free_margin;
      const double tentative_g = current_g + step + proximity_cost;

      NodeRecord& next_record = records[next];
      if (next_record.closed || tentative_g >= next_record.g) {
        continue;
      }
      next_record.g = tentative_g;
      next_record.f = tentative_g + (next_world - goal).norm();
      next_record.parent = current_entry.index;
      next_record.has_parent = true;
      open.push(QueueEntry{next_record.f, next});
    }
  }

  if (!found) {
    if (failure_reason) {
      std::ostringstream stream;
      stream << (expanded >= config_.max_expanded_nodes
                     ? "A* reached the node expansion limit"
                     : "A* open set was exhausted")
             << " (expanded=" << expanded
             << ", discovered=" << records.size()
             << ", resolution=" << resolution
             << ", bounds=[" << bounded_lower.transpose() << "]..["
             << bounded_upper.transpose() << "]).";
      *failure_reason = stream.str();
    }
    return false;
  }

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> reverse_path;
  GridIndex cursor = reached;
  reverse_path.push_back(goal);
  while (!(cursor == start_index)) {
    reverse_path.push_back(to_world(cursor));
    const auto it = records.find(cursor);
    if (it == records.end() || !it->second.has_parent) {
      if (failure_reason) *failure_reason = "A* parent chain is incomplete.";
      return false;
    }
    cursor = it->second.parent;
  }
  reverse_path.push_back(start);
  std::reverse(reverse_path.begin(), reverse_path.end());

  if (!config_.simplify_path || reverse_path.size() <= 2) {
    path = reverse_path;
    return true;
  }

  path.push_back(reverse_path.front());
  std::size_t anchor = 0;
  while (anchor + 1 < reverse_path.size()) {
    std::size_t farthest = reverse_path.size() - 1;
    while (farthest > anchor + 1 &&
           !lineIsSafe(reverse_path[anchor], reverse_path[farthest])) {
      --farthest;
    }
    path.push_back(reverse_path[farthest]);
    anchor = farthest;
  }
  return true;
}

}  // namespace bubble_planner
