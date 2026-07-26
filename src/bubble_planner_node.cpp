#include "bubble_planner/bubble_planner.hpp"

#include <Eigen/Core>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>

namespace bubble_planner {
namespace {

geometry_msgs::Quaternion yawQuaternion(double yaw) {
  geometry_msgs::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

double velocityYaw(const Eigen::Vector3d& velocity, double fallback) {
  if (velocity.head<2>().norm() < 0.05) {
    return fallback;
  }
  return std::atan2(velocity.y(), velocity.x());
}

void fillVector(const Eigen::Vector3d& source, geometry_msgs::Vector3& target) {
  target.x = source.x();
  target.y = source.y();
  target.z = source.z();
}

}  // namespace

class BubblePlannerNode {
 public:
  BubblePlannerNode()
      : nh_(), pnh_("~"), obstacle_index_(std::make_shared<ObstacleIndex>()) {
    loadParameters();
    planner_ = std::make_unique<BubblePlanner>(obstacle_index_, config_);

    cloud_sub_ = nh_.subscribe(cloud_topic_, 1,
                               &BubblePlannerNode::cloudCallback, this,
                               ros::TransportHints().tcpNoDelay());
    odom_sub_ = nh_.subscribe(odom_topic_, 20,
                              &BubblePlannerNode::odomCallback, this,
                              ros::TransportHints().tcpNoDelay());
    goal_sub_ = nh_.subscribe(goal_topic_, 2,
                              &BubblePlannerNode::goalCallback, this,
                              ros::TransportHints().tcpNoDelay());
    risk_sub_ = nh_.subscribe(risk_topic_, 5,
                              &BubblePlannerNode::riskCallback, this,
                              ros::TransportHints().tcpNoDelay());

    guide_path_pub_ = pnh_.advertise<nav_msgs::Path>("guide_path", 1, true);
    trajectory_path_pub_ = pnh_.advertise<nav_msgs::Path>("trajectory_path", 1, true);
    corridor_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("corridor", 1, true);
    trajectory_pub_ = pnh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        "trajectory", 1, true);
    desired_pose_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("reference/pose", 5);
    desired_velocity_pub_ = pnh_.advertise<geometry_msgs::TwistStamped>(
        "reference/velocity", 5);
    desired_acceleration_pub_ = pnh_.advertise<geometry_msgs::AccelStamped>(
        "reference/acceleration", 5);
    desired_jerk_pub_ = pnh_.advertise<geometry_msgs::Vector3Stamped>(
        "reference/jerk", 5);
    adjusted_goal_pub_ =
        pnh_.advertise<geometry_msgs::PoseStamped>("adjusted_goal", 1, true);
    status_pub_ = pnh_.advertise<std_msgs::String>("status", 5, true);

    replan_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, replan_rate_)),
                                    &BubblePlannerNode::replanTimer, this);
    command_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, command_rate_)),
                                     &BubblePlannerNode::commandTimer, this);
    publishStatus("Waiting for odometry, point cloud, and goal.");
  }

 private:
  void loadParameters() {
    pnh_.param<std::string>("frame_id", frame_id_, "world");
    pnh_.param<std::string>("cloud_topic", cloud_topic_, "/map_generator/global_cloud");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/odom_world");
    pnh_.param<std::string>("goal_topic", goal_topic_, "/move_base_simple/goal");
    pnh_.param<std::string>("risk_topic", risk_topic_, "/trajectory_risk_level");
    pnh_.param("override_goal_z", override_goal_z_, false);
    pnh_.param("goal_z", goal_z_, 1.5);
    pnh_.param("replan_rate", replan_rate_, 20.0);
    pnh_.param("command_rate", command_rate_, 100.0);
    pnh_.param("goal_tolerance", goal_tolerance_, 0.25);
    pnh_.param("goal_velocity_tolerance", goal_velocity_tolerance_, 0.25);
    pnh_.param("goal_adjustment/enabled", adjust_unsafe_goal_, true);
    pnh_.param("goal_adjustment/clearance_margin", goal_clearance_margin_, 0.0);
    pnh_.param("goal_adjustment/max_distance", goal_adjustment_max_distance_, 2.0);
    pnh_.param("goal_adjustment/search_step", goal_adjustment_search_step_, 0.10);

    pnh_.param("planning_horizon", config_.planning_horizon, 8.0);
    pnh_.param("distance_trigger_ratio", config_.distance_trigger_ratio, 0.4);
    pnh_.param("collision_check_horizon", config_.collision_check_horizon, 2.0);
    pnh_.param("collision_check_dt", config_.collision_check_dt, 0.05);
    pnh_.param("commit_time", config_.commit_time, 0.12);

    pnh_.param("astar/resolution", config_.astar.resolution, 0.25);
    pnh_.param("astar/clearance", config_.astar.clearance, 0.15);
    pnh_.param("astar/search_margin", config_.astar.search_margin, 2.0);
    pnh_.param("astar/min_z", config_.astar.min_z, 0.2);
    pnh_.param("astar/max_z", config_.astar.max_z, 5.0);
    pnh_.param("astar/proximity_weight", config_.astar.proximity_weight, 0.08);
    int max_nodes = static_cast<int>(config_.astar.max_expanded_nodes);
    pnh_.param("astar/max_expanded_nodes", max_nodes, 120000);
    config_.astar.max_expanded_nodes = static_cast<std::size_t>(std::max(1000, max_nodes));
    pnh_.param("astar/simplify_path", config_.astar.simplify_path, true);

    pnh_.param("corridor/paper_mode", config_.corridor.paper_mode, true);
    pnh_.param("corridor/random_seed", config_.corridor.random_seed, 7);
    pnh_.param("corridor/batch_size", config_.corridor.batch_size, 36);
    pnh_.param("corridor/max_spheres", config_.corridor.max_spheres, 18);
    pnh_.param("corridor/max_failed_batches", config_.corridor.max_failed_batches, 4);
    pnh_.param("corridor/drone_radius", config_.corridor.drone_radius, 0.30);
    pnh_.param("corridor/safety_margin", config_.corridor.safety_margin, 0.15);
    pnh_.param("corridor/min_radius", config_.corridor.min_radius, 0.20);
    pnh_.param("corridor/max_radius", config_.corridor.max_radius, 4.0);
    pnh_.param("corridor/overlap_depth", config_.corridor.overlap_depth, 0.12);
    pnh_.param("corridor/min_overlap_volume", config_.corridor.min_overlap_volume, 0.01);
    pnh_.param("corridor/guide_inside_margin",
               config_.corridor.guide_inside_margin, 0.05);
    pnh_.param("corridor/min_forward_progress", config_.corridor.min_forward_progress, 0.05);
    pnh_.param("corridor/nominal_speed", config_.corridor.nominal_speed, 3.0);
    pnh_.param("corridor/min_piece_time", config_.corridor.min_piece_time, 0.12);
    pnh_.param("corridor/rho_sphere_volume", config_.corridor.rho_sphere_volume, 1.0);
    pnh_.param("corridor/rho_overlap_volume", config_.corridor.rho_overlap_volume, 2.0);
    pnh_.param("corridor/rho_progress", config_.corridor.rho_progress, 0.0);
    pnh_.param("corridor/rho_guide_cost", config_.corridor.rho_guide_cost, 0.0);
    pnh_.param("corridor/adaptive_covariance", config_.corridor.adaptive_covariance, false);
    pnh_.param("corridor/longitudinal_sigma_scale",
               config_.corridor.longitudinal_sigma_scale, 0.50);
    pnh_.param("corridor/lateral_sigma_ratio",
               config_.corridor.lateral_sigma_ratio, 0.50);

    pnh_.param("rhc/enabled", config_.rhc.enabled, true);
    pnh_.param("rhc/reuse_distance", config_.rhc.reuse_distance, 3.0);
    pnh_.param("rhc/max_reused_spheres", config_.rhc.max_reused_spheres, 5);
    pnh_.param("rhc/radius_revalidation_margin",
               config_.rhc.radius_revalidation_margin, 0.02);

    pnh_.param("minco/max_velocity", config_.minco.max_velocity, 6.0);
    pnh_.param("minco/max_acceleration", config_.minco.max_acceleration, 8.0);
    pnh_.param("minco/max_jerk", config_.minco.max_jerk, 25.0);
    pnh_.param("minco/nominal_speed", config_.minco.nominal_speed, 3.0);
    pnh_.param("minco/min_piece_time", config_.minco.min_piece_time, 0.12);
    pnh_.param("minco/weight_time", config_.minco.weight_time, 20.0);
    pnh_.param("minco/weight_corridor", config_.minco.weight_corridor, 8000.0);
    pnh_.param("minco/weight_velocity", config_.minco.weight_velocity, 2000.0);
    pnh_.param("minco/weight_acceleration", config_.minco.weight_acceleration, 2000.0);
    pnh_.param("minco/weight_jerk", config_.minco.weight_jerk, 50.0);
    pnh_.param("minco/weight_waypoint", config_.minco.weight_waypoint, 5000.0);
    pnh_.param("minco/smoothing_mu", config_.minco.smoothing_mu, 0.02);
    pnh_.param("minco/quadrature_intervals", config_.minco.quadrature_intervals, 8);
    pnh_.param("minco/validation_samples_per_piece",
               config_.minco.validation_samples_per_piece, 40);
    pnh_.param("minco/max_iterations", config_.minco.max_iterations, 45);
    pnh_.param("minco/lbfgs_memory", config_.minco.lbfgs_memory, 12);
    pnh_.param("minco/relative_cost_tolerance",
               config_.minco.relative_cost_tolerance, 1.0e-5);
    pnh_.param("minco/gradient_epsilon", config_.minco.gradient_epsilon, 1.0e-5);
    pnh_.param("minco/finite_difference_step",
               config_.minco.finite_difference_step, 2.0e-5);
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& message) {
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*message, *cloud);
    obstacle_index_->update(cloud);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    odom_state_.position = Eigen::Vector3d(message->pose.pose.position.x,
                                           message->pose.pose.position.y,
                                           message->pose.pose.position.z);
    odom_state_.velocity = Eigen::Vector3d(message->twist.twist.linear.x,
                                           message->twist.twist.linear.y,
                                           message->twist.twist.linear.z);
    odom_state_.acceleration.setZero();
    odom_state_.jerk.setZero();
    odom_ready_ = true;
  }

  bool adjustGoalForReplanning(const Eigen::Vector3d& requested,
                               Eigen::Vector3d& adjusted,
                               double& requested_distance) const {
    adjusted = requested;
    requested_distance = std::numeric_limits<double>::infinity();
    if (!obstacle_index_->ready() ||
        !obstacle_index_->nearest(requested, requested_distance, nullptr)) {
      return true;
    }

    // A terminal point must not merely satisfy the vehicle clearance.  It must
    // also leave enough room to construct the first corridor sphere of the
    // next plan; otherwise one successful flight can strand the replanner.
    const double required_clearance = std::max(
        config_.astar.clearance,
        config_.corridor.drone_radius + config_.corridor.safety_margin +
            config_.corridor.min_radius + goal_clearance_margin_);
    if (requested_distance >= required_clearance) {
      return true;
    }
    if (!adjust_unsafe_goal_) {
      return false;
    }

    Eigen::Vector3d nearest_point = requested;
    double ignored_distance = requested_distance;
    obstacle_index_->nearest(requested, ignored_distance, &nearest_point);
    const Eigen::Vector2d outward = (requested - nearest_point).head<2>();
    const double base_angle = outward.norm() > 1.0e-6
                                  ? std::atan2(outward.y(), outward.x())
                                  : 0.0;
    const double step = std::max(0.05, goal_adjustment_search_step_);
    const double max_distance = std::max(step, goal_adjustment_max_distance_);
    const double two_pi = 2.0 * std::acos(-1.0);
    for (double radius = step; radius <= max_distance + 1.0e-9; radius += step) {
      const int samples =
          std::max(16, static_cast<int>(std::ceil(two_pi * radius / step)));
      for (int i = 0; i < samples; ++i) {
        const double angle = base_angle + two_pi * static_cast<double>(i) /
                                            static_cast<double>(samples);
        Eigen::Vector3d candidate = requested;
        candidate.x() += radius * std::cos(angle);
        candidate.y() += radius * std::sin(angle);
        if (candidate.z() < config_.astar.min_z ||
            candidate.z() > config_.astar.max_z) {
          continue;
        }
        double candidate_distance = std::numeric_limits<double>::infinity();
        if (obstacle_index_->nearest(candidate, candidate_distance, nullptr) &&
            candidate_distance >= required_clearance) {
          adjusted = candidate;
          return true;
        }
      }
    }
    return false;
  }

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    const Eigen::Vector3d requested(
        message->pose.position.x,
        message->pose.position.y,
        override_goal_z_ ? goal_z_ : message->pose.position.z);
    Eigen::Vector3d adjusted;
    double requested_distance = std::numeric_limits<double>::infinity();
    if (!adjustGoalForReplanning(requested, adjusted, requested_distance)) {
      const double required_clearance = std::max(
          config_.astar.clearance,
          config_.corridor.drone_radius + config_.corridor.safety_margin +
              config_.corridor.min_radius + goal_clearance_margin_);
      std::ostringstream stream;
      stream << "Rejected unsafe goal: nearest obstacle=" << requested_distance
             << " m, required=" << required_clearance
             << " m, and no safe point was found within "
             << goal_adjustment_max_distance_ << " m.";
      publishStatus(stream.str());
      ROS_WARN_STREAM(stream.str());
      return;
    }

    const double adjustment = (adjusted - requested).norm();
    if (adjustment > 1.0e-6) {
      ROS_WARN_STREAM("Goal is too close to an obstacle and was shifted by "
                      << adjustment << " m from [" << requested.transpose()
                      << "] to [" << adjusted.transpose()
                      << "] so the next plan can start safely.");
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      global_goal_ = adjusted;
      goal_ready_ = true;
      force_replan_ = true;
      risk_level_ = 0;
    }

    geometry_msgs::PoseStamped adjusted_message = *message;
    adjusted_message.header.stamp = ros::Time::now();
    adjusted_message.header.frame_id = frame_id_;
    adjusted_message.pose.position.x = adjusted.x();
    adjusted_message.pose.position.y = adjusted.y();
    adjusted_message.pose.position.z = adjusted.z();
    adjusted_goal_pub_.publish(adjusted_message);
  }

  void riskCallback(const std_msgs::UInt8::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    risk_level_ = message->data;
    if (risk_level_ > 0) {
      force_replan_ = true;
    }
  }

  bool currentTrajectoryUnsafe(const ros::Time& now) const {
    if (!has_current_plan_) {
      return true;
    }
    const double elapsed = (now - current_start_time_).toSec();
    const double total = current_plan_.trajectory.getTotalDuration();
    if (elapsed < 0.0 || elapsed >= total) {
      return true;
    }
    const double clearance = config_.corridor.drone_radius + config_.corridor.safety_margin;
    const double end = std::min(total, elapsed + config_.collision_check_horizon);
    for (double t = elapsed; t <= end + 1.0e-9; t += config_.collision_check_dt) {
      if (!obstacle_index_->isSafe(current_plan_.trajectory.getPos(std::min(t, end)),
                                   clearance)) {
        return true;
      }
    }
    return false;
  }

  void replanTimer(const ros::TimerEvent&) {
    BoundaryState odom;
    Eigen::Vector3d global_goal;
    bool goal_ready = false;
    bool odom_ready = false;
    bool forced = false;
    std::uint8_t risk = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      odom = odom_state_;
      global_goal = global_goal_;
      goal_ready = goal_ready_;
      odom_ready = odom_ready_;
      forced = force_replan_;
      risk = risk_level_;
    }
    if (!goal_ready || !odom_ready || !obstacle_index_->ready()) {
      return;
    }

    const ros::Time now = ros::Time::now();
    const double clearance = config_.corridor.drone_radius +
                             config_.corridor.safety_margin;
    const bool safely_holding_goal =
        has_current_plan_ &&
        (odom.position - global_goal).norm() <= goal_tolerance_ &&
        odom.velocity.norm() <= goal_velocity_tolerance_ &&
        risk == 0 &&
        obstacle_index_->isSafe(odom.position, clearance);
    if (safely_holding_goal) {
      std::lock_guard<std::mutex> lock(mutex_);
      force_replan_ = false;
      return;
    }

    bool trigger = forced || !has_current_plan_;
    if (has_current_plan_) {
      trigger = trigger ||
          (odom.position - last_plan_position_).norm() >=
              config_.distance_trigger_ratio * config_.planning_horizon;
      trigger = trigger || risk > 0 || currentTrajectoryUnsafe(now);
    }
    if (!trigger || planning_) {
      return;
    }

    planning_ = true;
    BoundaryState start = odom;
    ros::Time switch_time = now;
    if (has_current_plan_) {
      const double elapsed = (now - current_start_time_).toSec();
      const double future = elapsed + config_.commit_time;
      if (future > 0.0 && future < current_plan_.trajectory.getTotalDuration()) {
        start.position = current_plan_.trajectory.getPos(future);
        start.velocity = current_plan_.trajectory.getVel(future);
        start.acceleration = current_plan_.trajectory.getAcc(future);
        start.jerk = current_plan_.trajectory.getJer(future);
        switch_time = now + ros::Duration(config_.commit_time);
      }
    }

    BoundaryState local_goal;
    Eigen::Vector3d goal_delta = global_goal - start.position;
    const double goal_distance = goal_delta.norm();
    if (goal_distance > config_.planning_horizon) {
      goal_delta.normalize();
      local_goal.position = start.position + config_.planning_horizon * goal_delta;
      local_goal.velocity = config_.corridor.nominal_speed * goal_delta;
    } else {
      local_goal.position = global_goal;
      local_goal.velocity.setZero();
    }
    local_goal.acceleration.setZero();
    local_goal.jerk.setZero();

    PlanResult planned;
    const PlanResult* previous = has_current_plan_ ? &current_plan_ : nullptr;
    const bool success = planner_->plan(start, local_goal, planned, previous);
    if (success) {
      if (switch_time > now + ros::Duration(1.0e-4) && has_current_plan_) {
        pending_plan_ = planned;
        pending_start_time_ = switch_time;
        has_pending_plan_ = true;
      } else {
        current_plan_ = planned;
        current_start_time_ = now;
        has_current_plan_ = true;
        has_pending_plan_ = false;
      }
      last_plan_position_ = odom.position;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        force_replan_ = false;
        risk_level_ = 0;
      }
      publishPlan(planned, switch_time);
      publishStatus("Planning succeeded: " + planned.message);
    } else {
      publishStatus("Planning failed: " + planned.message);
      ROS_WARN_STREAM_THROTTLE(1.0, "Bubble Planner failed: " << planned.message);
    }
    planning_ = false;
  }

  void commandTimer(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    if (has_pending_plan_ && now >= pending_start_time_) {
      current_plan_ = pending_plan_;
      current_start_time_ = pending_start_time_;
      has_current_plan_ = true;
      has_pending_plan_ = false;
    }
    if (!has_current_plan_) {
      return;
    }
    double elapsed = (now - current_start_time_).toSec();
    if (elapsed < 0.0) {
      return;
    }
    const double total = current_plan_.trajectory.getTotalDuration();
    elapsed = std::min(elapsed, total);
    const Eigen::Vector3d position = current_plan_.trajectory.getPos(elapsed);
    const Eigen::Vector3d velocity = current_plan_.trajectory.getVel(elapsed);
    const Eigen::Vector3d acceleration = current_plan_.trajectory.getAcc(elapsed);
    const Eigen::Vector3d jerk = current_plan_.trajectory.getJer(elapsed);
    const double yaw = velocityYaw(velocity, last_yaw_);
    last_yaw_ = yaw;

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = position.x();
    pose.pose.position.y = position.y();
    pose.pose.position.z = position.z();
    pose.pose.orientation = yawQuaternion(yaw);
    desired_pose_pub_.publish(pose);

    geometry_msgs::TwistStamped velocity_message;
    velocity_message.header = pose.header;
    fillVector(velocity, velocity_message.twist.linear);
    desired_velocity_pub_.publish(velocity_message);

    geometry_msgs::AccelStamped acceleration_message;
    acceleration_message.header = pose.header;
    fillVector(acceleration, acceleration_message.accel.linear);
    desired_acceleration_pub_.publish(acceleration_message);

    geometry_msgs::Vector3Stamped jerk_message;
    jerk_message.header = pose.header;
    fillVector(jerk, jerk_message.vector);
    desired_jerk_pub_.publish(jerk_message);
  }

  void publishPlan(const PlanResult& plan, const ros::Time& start_time) {
    const ros::Time stamp = ros::Time::now();
    nav_msgs::Path guide;
    guide.header.frame_id = frame_id_;
    guide.header.stamp = stamp;
    for (const auto& point : plan.corridor.guide_path) {
      geometry_msgs::PoseStamped pose;
      pose.header = guide.header;
      pose.pose.position.x = point.x();
      pose.pose.position.y = point.y();
      pose.pose.position.z = point.z();
      pose.pose.orientation.w = 1.0;
      guide.poses.push_back(pose);
    }
    guide_path_pub_.publish(guide);

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker clear;
    clear.header = guide.header;
    clear.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear);
    int marker_id = 0;
    for (const auto& sphere : plan.corridor.spheres) {
      visualization_msgs::Marker marker;
      marker.header = guide.header;
      marker.ns = "bubble_corridor";
      marker.id = marker_id++;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = sphere.center.x();
      marker.pose.position.y = sphere.center.y();
      marker.pose.position.z = sphere.center.z();
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 2.0 * sphere.radius;
      marker.scale.y = 2.0 * sphere.radius;
      marker.scale.z = 2.0 * sphere.radius;
      // Keep the obstacle cloud and trajectory readable through overlapping
      // spheres.  Reused spheres use green; newly sampled spheres use cyan.
      marker.color.a = 0.13;
      if (sphere.reused) {
        marker.color.r = 0.15;
        marker.color.g = 1.0;
        marker.color.b = 0.35;
      } else {
        marker.color.r = 0.0;
        marker.color.g = 0.65;
        marker.color.b = 1.0;
      }
      marker_array.markers.push_back(marker);
    }
    corridor_pub_.publish(marker_array);

    nav_msgs::Path trajectory_path;
    trajectory_path.header = guide.header;
    trajectory_msgs::MultiDOFJointTrajectory trajectory_message;
    trajectory_message.header.frame_id = frame_id_;
    trajectory_message.header.stamp = start_time;
    trajectory_message.joint_names.push_back("base_link");
    const double total = plan.trajectory.getTotalDuration();
    const double sample_dt = 0.02;
    double yaw = last_yaw_;
    for (double t = 0.0; t <= total + 1.0e-9; t += sample_dt) {
      const double query_time = std::min(t, total);
      const Eigen::Vector3d position = plan.trajectory.getPos(query_time);
      const Eigen::Vector3d velocity = plan.trajectory.getVel(query_time);
      const Eigen::Vector3d acceleration = plan.trajectory.getAcc(query_time);
      yaw = velocityYaw(velocity, yaw);

      geometry_msgs::PoseStamped pose;
      pose.header = trajectory_path.header;
      pose.pose.position.x = position.x();
      pose.pose.position.y = position.y();
      pose.pose.position.z = position.z();
      pose.pose.orientation = yawQuaternion(yaw);
      trajectory_path.poses.push_back(pose);

      trajectory_msgs::MultiDOFJointTrajectoryPoint point;
      geometry_msgs::Transform transform;
      transform.translation.x = position.x();
      transform.translation.y = position.y();
      transform.translation.z = position.z();
      transform.rotation = pose.pose.orientation;
      point.transforms.push_back(transform);
      geometry_msgs::Twist velocity_twist;
      fillVector(velocity, velocity_twist.linear);
      point.velocities.push_back(velocity_twist);
      geometry_msgs::Twist acceleration_twist;
      fillVector(acceleration, acceleration_twist.linear);
      point.accelerations.push_back(acceleration_twist);
      point.time_from_start = ros::Duration(query_time);
      trajectory_message.points.push_back(point);
    }
    trajectory_path_pub_.publish(trajectory_path);
    trajectory_pub_.publish(trajectory_message);
  }

  void publishStatus(const std::string& text) {
    std_msgs::String status;
    status.data = text;
    status_pub_.publish(status);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  PlannerConfig config_;
  std::shared_ptr<ObstacleIndex> obstacle_index_;
  std::unique_ptr<BubblePlanner> planner_;

  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber risk_sub_;
  ros::Publisher guide_path_pub_;
  ros::Publisher trajectory_path_pub_;
  ros::Publisher corridor_pub_;
  ros::Publisher trajectory_pub_;
  ros::Publisher desired_pose_pub_;
  ros::Publisher desired_velocity_pub_;
  ros::Publisher desired_acceleration_pub_;
  ros::Publisher desired_jerk_pub_;
  ros::Publisher adjusted_goal_pub_;
  ros::Publisher status_pub_;
  ros::Timer replan_timer_;
  ros::Timer command_timer_;

  std::string frame_id_;
  std::string cloud_topic_;
  std::string odom_topic_;
  std::string goal_topic_;
  std::string risk_topic_;
  double replan_rate_{20.0};
  double command_rate_{100.0};
  double goal_tolerance_{0.25};
  double goal_velocity_tolerance_{0.25};
  bool adjust_unsafe_goal_{true};
  double goal_clearance_margin_{0.0};
  double goal_adjustment_max_distance_{2.0};
  double goal_adjustment_search_step_{0.10};
  bool override_goal_z_{false};
  double goal_z_{1.5};

  mutable std::mutex mutex_;
  BoundaryState odom_state_;
  Eigen::Vector3d global_goal_{Eigen::Vector3d::Zero()};
  bool odom_ready_{false};
  bool goal_ready_{false};
  bool force_replan_{false};
  std::uint8_t risk_level_{0};

  PlanResult current_plan_;
  PlanResult pending_plan_;
  bool has_current_plan_{false};
  bool has_pending_plan_{false};
  bool planning_{false};
  ros::Time current_start_time_;
  ros::Time pending_start_time_;
  Eigen::Vector3d last_plan_position_{Eigen::Vector3d::Zero()};
  double last_yaw_{0.0};
};

}  // namespace bubble_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "bubble_planner_node");
  bubble_planner::BubblePlannerNode node;
  ros::spin();
  return 0;
}
