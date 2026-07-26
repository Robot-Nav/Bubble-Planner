#pragma once

#include <Eigen/Core>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <memory>
#include <mutex>

namespace bubble_planner {

class ObstacleIndex {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ObstacleIndex();

  void update(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
  [[nodiscard]] bool ready() const;
  [[nodiscard]] std::size_t size() const;

  bool nearest(const Eigen::Vector3d& query,
               double& distance,
               Eigen::Vector3d* nearest_point = nullptr) const;

  [[nodiscard]] bool isSafe(const Eigen::Vector3d& query,
                            double clearance) const;

 private:
  mutable std::mutex mutex_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree_;
};

}  // namespace bubble_planner
