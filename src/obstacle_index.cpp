#include "bubble_planner/obstacle_index.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace bubble_planner {

ObstacleIndex::ObstacleIndex()
    : cloud_(new pcl::PointCloud<pcl::PointXYZ>()),
      tree_(new pcl::KdTreeFLANN<pcl::PointXYZ>()) {}

void ObstacleIndex::update(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
  if (!cloud) {
    return;
  }
  auto filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  filtered->reserve(cloud->size());
  for (const auto& point : cloud->points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      filtered->push_back(point);
    }
  }
  filtered->width = static_cast<std::uint32_t>(filtered->size());
  filtered->height = 1;
  filtered->is_dense = true;

  std::lock_guard<std::mutex> lock(mutex_);
  cloud_ = filtered;
  tree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  if (!cloud_->empty()) {
    tree_->setInputCloud(cloud_);
  }
}

bool ObstacleIndex::ready() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cloud_ && !cloud_->empty();
}

std::size_t ObstacleIndex::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cloud_ ? cloud_->size() : 0U;
}

bool ObstacleIndex::nearest(const Eigen::Vector3d& query,
                            double& distance,
                            Eigen::Vector3d* nearest_point) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!cloud_ || cloud_->empty() || !tree_) {
    distance = std::numeric_limits<double>::infinity();
    return false;
  }

  const pcl::PointXYZ point(static_cast<float>(query.x()),
                            static_cast<float>(query.y()),
                            static_cast<float>(query.z()));
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  if (tree_->nearestKSearch(point, 1, indices, squared_distances) <= 0) {
    distance = std::numeric_limits<double>::infinity();
    return false;
  }
  distance = std::sqrt(static_cast<double>(squared_distances.front()));
  if (nearest_point != nullptr) {
    const auto& nearest_cloud_point = cloud_->points[indices.front()];
    *nearest_point = Eigen::Vector3d(nearest_cloud_point.x,
                                     nearest_cloud_point.y,
                                     nearest_cloud_point.z);
  }
  return true;
}

bool ObstacleIndex::isSafe(const Eigen::Vector3d& query, double clearance) const {
  double distance = std::numeric_limits<double>::infinity();
  if (!nearest(query, distance, nullptr)) {
    return true;
  }
  return distance >= clearance;
}

}  // namespace bubble_planner
