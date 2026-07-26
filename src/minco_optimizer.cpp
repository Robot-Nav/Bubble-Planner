#include "bubble_planner/minco_optimizer.hpp"

#include "bubble_planner/geometry.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

namespace bubble_planner {
namespace {

struct PieceState {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
  Eigen::Vector3d snap{Eigen::Vector3d::Zero()};
};

PieceState evaluatePolynomial(const Eigen::Matrix<double, 8, 3>& coefficients,
                              double time) {
  PieceState state;
  std::array<double, 8> powers{};
  powers[0] = 1.0;
  for (int k = 1; k < 8; ++k) {
    powers[k] = powers[k - 1] * time;
  }
  for (int k = 0; k < 8; ++k) {
    state.position += coefficients.row(k).transpose() * powers[k];
    if (k >= 1) {
      state.velocity += coefficients.row(k).transpose() *
                        (static_cast<double>(k) * powers[k - 1]);
    }
    if (k >= 2) {
      state.acceleration += coefficients.row(k).transpose() *
          (static_cast<double>(k * (k - 1)) * powers[k - 2]);
    }
    if (k >= 3) {
      state.jerk += coefficients.row(k).transpose() *
          (static_cast<double>(k * (k - 1) * (k - 2)) * powers[k - 3]);
    }
    if (k >= 4) {
      state.snap += coefficients.row(k).transpose() *
          (static_cast<double>(k * (k - 1) * (k - 2) * (k - 3)) *
           powers[k - 4]);
    }
  }
  return state;
}

template <typename GradientType>
void accumulateCoefficientGradient(
    double time,
    const Eigen::Vector3d& grad_position,
    const Eigen::Vector3d& grad_velocity,
    const Eigen::Vector3d& grad_acceleration,
    const Eigen::Vector3d& grad_jerk,
    GradientType& gradient) {
  std::array<double, 8> powers{};
  powers[0] = 1.0;
  for (int k = 1; k < 8; ++k) {
    powers[k] = powers[k - 1] * time;
  }
  for (int k = 0; k < 8; ++k) {
    Eigen::Vector3d row_gradient = grad_position * powers[k];
    if (k >= 1) {
      row_gradient += grad_velocity *
          (static_cast<double>(k) * powers[k - 1]);
    }
    if (k >= 2) {
      row_gradient += grad_acceleration *
          (static_cast<double>(k * (k - 1)) * powers[k - 2]);
    }
    if (k >= 3) {
      row_gradient += grad_jerk *
          (static_cast<double>(k * (k - 1) * (k - 2)) * powers[k - 3]);
    }
    gradient.row(k) += row_gradient.transpose();
  }
}

}  // namespace

MincoOptimizer::MincoOptimizer(MincoConfig config) : config_(std::move(config)) {}

void MincoOptimizer::decodeVariables(const Eigen::VectorXd& variables,
                                     Eigen::MatrixXd& waypoints,
                                     Eigen::VectorXd& times) const {
  const int waypoint_count = std::max(0, piece_count_ - 1);
  waypoints.resize(3, waypoint_count);
  for (int i = 0; i < waypoint_count; ++i) {
    waypoints.col(i) = variables.segment<3>(3 * i);
  }
  times.resize(piece_count_);
  const int time_offset = 3 * waypoint_count;
  for (int i = 0; i < piece_count_; ++i) {
    const double log_time = clamp(variables(time_offset + i), -6.0, 6.0);
    times(i) = std::exp(log_time);
  }
}

double MincoOptimizer::evaluateObjective(const Eigen::VectorXd& variables,
                                         Trajectory<7>* trajectory) const {
  Eigen::MatrixXd waypoints;
  Eigen::VectorXd times;
  decodeVariables(variables, waypoints, times);
  minco_.setParameters(waypoints, times);

  Trajectory<7> local_trajectory;
  minco_.getTrajectory(local_trajectory);
  double energy = 0.0;
  minco_.getEnergy(energy);
  double cost = energy + config_.weight_time * times.sum();

  for (int i = 0; i < piece_count_; ++i) {
    cost += config_.weight_time * 5.0 *
            smoothBarrier(config_.min_piece_time - times(i), config_.smoothing_mu);
  }

  const int intervals = std::max(2, config_.quadrature_intervals);
  for (int piece = 0; piece < piece_count_; ++piece) {
    const double duration = times(piece);
    const double step = duration / static_cast<double>(intervals);
    for (int sample = 0; sample <= intervals; ++sample) {
      const double weight = (sample == 0 || sample == intervals) ? 0.5 : 1.0;
      const double t = std::min(duration, step * sample);
      const Eigen::Vector3d position = local_trajectory[piece].getPos(t);
      const Eigen::Vector3d velocity = local_trajectory[piece].getVel(t);
      const Eigen::Vector3d acceleration = local_trajectory[piece].getAcc(t);
      const Eigen::Vector3d jerk = local_trajectory[piece].getJer(t);

      const double corridor_violation =
          (position - corridor_[piece].center).squaredNorm() -
          corridor_[piece].radius * corridor_[piece].radius;
      const double velocity_violation = velocity.squaredNorm() -
                                         config_.max_velocity * config_.max_velocity;
      const double acceleration_violation = acceleration.squaredNorm() -
          config_.max_acceleration * config_.max_acceleration;
      const double jerk_violation = jerk.squaredNorm() -
                                     config_.max_jerk * config_.max_jerk;
      cost += weight * step * (
          config_.weight_corridor * smoothBarrier(corridor_violation, config_.smoothing_mu) +
          config_.weight_velocity * smoothBarrier(velocity_violation, config_.smoothing_mu) +
          config_.weight_acceleration * smoothBarrier(acceleration_violation, config_.smoothing_mu) +
          config_.weight_jerk * smoothBarrier(jerk_violation, config_.smoothing_mu));
    }
  }

  for (int i = 0; i + 1 < piece_count_; ++i) {
    const Eigen::Vector3d waypoint = waypoints.col(i);
    const double first_violation = (waypoint - corridor_[i].center).squaredNorm() -
                                   corridor_[i].radius * corridor_[i].radius;
    const double second_violation = (waypoint - corridor_[i + 1].center).squaredNorm() -
                                    corridor_[i + 1].radius * corridor_[i + 1].radius;
    cost += config_.weight_waypoint *
            (smoothBarrier(first_violation, config_.smoothing_mu) +
             smoothBarrier(second_violation, config_.smoothing_mu));
  }

  if (trajectory != nullptr) {
    *trajectory = local_trajectory;
  }
  return std::isfinite(cost) ? cost : std::numeric_limits<double>::max() / 16.0;
}

double MincoOptimizer::evaluateObjectiveAndGradient(
    const Eigen::VectorXd& variables,
    Eigen::VectorXd& gradient) const {
  Eigen::MatrixXd waypoints;
  Eigen::VectorXd times;
  decodeVariables(variables, waypoints, times);
  minco_.setParameters(waypoints, times);

  double energy = 0.0;
  minco_.getEnergy(energy);
  double cost = energy + config_.weight_time * times.sum();

  Eigen::MatrixX3d partial_coefficients;
  Eigen::VectorXd partial_times;
  minco_.getEnergyPartialGradByCoeffs(partial_coefficients);
  minco_.getEnergyPartialGradByTimes(partial_times);
  partial_times.array() += config_.weight_time;

  Eigen::MatrixX3d coefficients = minco_.getCoeffs();
  Eigen::Matrix3Xd explicit_waypoint_gradient =
      Eigen::Matrix3Xd::Zero(3, std::max(0, piece_count_ - 1));

  for (int i = 0; i < piece_count_; ++i) {
    const double violation = config_.min_piece_time - times(i);
    cost += config_.weight_time * 5.0 *
            smoothBarrier(violation, config_.smoothing_mu);
    partial_times(i) -= config_.weight_time * 5.0 *
                        smoothBarrierDerivative(violation, config_.smoothing_mu);
  }

  const int intervals = std::max(2, config_.quadrature_intervals);
  for (int piece = 0; piece < piece_count_; ++piece) {
    const double duration = times(piece);
    const double step = duration / static_cast<double>(intervals);
    const Eigen::Matrix<double, 8, 3> piece_coefficients =
        coefficients.block<8, 3>(8 * piece, 0);
    auto coefficient_gradient = partial_coefficients.block<8, 3>(8 * piece, 0);

    for (int sample = 0; sample <= intervals; ++sample) {
      const double quadrature_weight =
          (sample == 0 || sample == intervals) ? 0.5 : 1.0;
      const double alpha = static_cast<double>(sample) /
                           static_cast<double>(intervals);
      const double time = alpha * duration;
      const PieceState state = evaluatePolynomial(piece_coefficients, time);

      const Eigen::Vector3d offset = state.position - corridor_[piece].center;
      const double corridor_violation = offset.squaredNorm() -
          corridor_[piece].radius * corridor_[piece].radius;
      const double velocity_violation = state.velocity.squaredNorm() -
          config_.max_velocity * config_.max_velocity;
      const double acceleration_violation = state.acceleration.squaredNorm() -
          config_.max_acceleration * config_.max_acceleration;
      const double jerk_violation = state.jerk.squaredNorm() -
          config_.max_jerk * config_.max_jerk;

      const double corridor_barrier =
          smoothBarrier(corridor_violation, config_.smoothing_mu);
      const double velocity_barrier =
          smoothBarrier(velocity_violation, config_.smoothing_mu);
      const double acceleration_barrier =
          smoothBarrier(acceleration_violation, config_.smoothing_mu);
      const double jerk_barrier =
          smoothBarrier(jerk_violation, config_.smoothing_mu);
      const double penalty =
          config_.weight_corridor * corridor_barrier +
          config_.weight_velocity * velocity_barrier +
          config_.weight_acceleration * acceleration_barrier +
          config_.weight_jerk * jerk_barrier;
      cost += quadrature_weight * step * penalty;

      Eigen::Vector3d grad_position = 2.0 * config_.weight_corridor *
          smoothBarrierDerivative(corridor_violation, config_.smoothing_mu) * offset;
      Eigen::Vector3d grad_velocity = 2.0 * config_.weight_velocity *
          smoothBarrierDerivative(velocity_violation, config_.smoothing_mu) * state.velocity;
      Eigen::Vector3d grad_acceleration = 2.0 * config_.weight_acceleration *
          smoothBarrierDerivative(acceleration_violation, config_.smoothing_mu) *
          state.acceleration;
      Eigen::Vector3d grad_jerk = 2.0 * config_.weight_jerk *
          smoothBarrierDerivative(jerk_violation, config_.smoothing_mu) * state.jerk;

      const double integration_scale = quadrature_weight * step;
      accumulateCoefficientGradient(time,
                                    integration_scale * grad_position,
                                    integration_scale * grad_velocity,
                                    integration_scale * grad_acceleration,
                                    integration_scale * grad_jerk,
                                    coefficient_gradient);

      const double penalty_time_derivative = alpha * (
          grad_position.dot(state.velocity) +
          grad_velocity.dot(state.acceleration) +
          grad_acceleration.dot(state.jerk) +
          grad_jerk.dot(state.snap));
      partial_times(piece) += quadrature_weight *
          (penalty / static_cast<double>(intervals) +
           step * penalty_time_derivative);
    }
  }

  for (int i = 0; i + 1 < piece_count_; ++i) {
    const Eigen::Vector3d waypoint = waypoints.col(i);
    const Eigen::Vector3d first_offset = waypoint - corridor_[i].center;
    const Eigen::Vector3d second_offset = waypoint - corridor_[i + 1].center;
    const double first_violation = first_offset.squaredNorm() -
                                   corridor_[i].radius * corridor_[i].radius;
    const double second_violation = second_offset.squaredNorm() -
                                    corridor_[i + 1].radius * corridor_[i + 1].radius;
    cost += config_.weight_waypoint *
            (smoothBarrier(first_violation, config_.smoothing_mu) +
             smoothBarrier(second_violation, config_.smoothing_mu));
    explicit_waypoint_gradient.col(i) += 2.0 * config_.weight_waypoint * (
        smoothBarrierDerivative(first_violation, config_.smoothing_mu) * first_offset +
        smoothBarrierDerivative(second_violation, config_.smoothing_mu) * second_offset);
  }

  Eigen::Matrix3Xd waypoint_gradient;
  Eigen::VectorXd time_gradient;
  minco_.propogateGrad(partial_coefficients,
                       partial_times,
                       waypoint_gradient,
                       time_gradient);
  waypoint_gradient += explicit_waypoint_gradient;

  gradient.setZero(variables.size());
  const int waypoint_count = piece_count_ - 1;
  for (int i = 0; i < waypoint_count; ++i) {
    gradient.segment<3>(3 * i) = waypoint_gradient.col(i);
  }
  const int time_offset = 3 * waypoint_count;
  for (int i = 0; i < piece_count_; ++i) {
    const double log_time = variables(time_offset + i);
    gradient(time_offset + i) =
        (log_time > -6.0 && log_time < 6.0) ? time_gradient(i) * times(i) : 0.0;
  }

  if (!std::isfinite(cost) || !gradient.allFinite()) {
    gradient.setZero();
    return std::numeric_limits<double>::max() / 16.0;
  }
  return cost;
}

double MincoOptimizer::objectiveCallback(void* instance,
                                         const Eigen::VectorXd& variables,
                                         Eigen::VectorXd& gradient) {
  auto* optimizer = static_cast<MincoOptimizer*>(instance);
  return optimizer->evaluateObjectiveAndGradient(variables, gradient);
}

bool MincoOptimizer::validate(
    const Trajectory<7>& trajectory,
    const std::vector<Sphere, Eigen::aligned_allocator<Sphere>>& corridor,
    std::string* failure_reason) const {
  if (trajectory.getPieceNum() != static_cast<int>(corridor.size())) {
    if (failure_reason) *failure_reason = "Trajectory piece count does not match corridor.";
    return false;
  }
  const int samples = std::max(8, config_.validation_samples_per_piece);
  for (int piece = 0; piece < trajectory.getPieceNum(); ++piece) {
    const double duration = trajectory[piece].getDuration();
    if (!(duration > 0.0) || !std::isfinite(duration)) {
      if (failure_reason) *failure_reason = "A trajectory piece has invalid duration.";
      return false;
    }
    for (int sample = 0; sample <= samples; ++sample) {
      const double t = duration * static_cast<double>(sample) / static_cast<double>(samples);
      const Eigen::Vector3d position = trajectory[piece].getPos(t);
      const Eigen::Vector3d velocity = trajectory[piece].getVel(t);
      const Eigen::Vector3d acceleration = trajectory[piece].getAcc(t);
      const Eigen::Vector3d jerk = trajectory[piece].getJer(t);
      if (!corridor[piece].contains(position, 1.0e-3)) {
        if (failure_reason) {
          std::ostringstream stream;
          stream << "Trajectory leaves sphere " << piece << ".";
          *failure_reason = stream.str();
        }
        return false;
      }
      if (velocity.norm() > 1.02 * config_.max_velocity ||
          acceleration.norm() > 1.02 * config_.max_acceleration ||
          jerk.norm() > 1.05 * config_.max_jerk) {
        if (failure_reason) {
          std::ostringstream stream;
          stream << "Dynamic limit violation in piece " << piece << ".";
          *failure_reason = stream.str();
        }
        return false;
      }
    }
  }
  return true;
}

bool MincoOptimizer::optimize(
    const BoundaryState& start,
    const BoundaryState& goal,
    const std::vector<Sphere, Eigen::aligned_allocator<Sphere>>& corridor,
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& initial_waypoints,
    const std::vector<double>& initial_times,
    Trajectory<7>& trajectory,
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& optimized_waypoints,
    std::vector<double>& optimized_times,
    std::string* failure_reason) {
  if (corridor.empty()) {
    if (failure_reason) *failure_reason = "MINCO received an empty corridor.";
    return false;
  }
  piece_count_ = static_cast<int>(corridor.size());
  if (static_cast<int>(initial_waypoints.size()) != piece_count_ - 1 ||
      static_cast<int>(initial_times.size()) != piece_count_) {
    if (failure_reason) *failure_reason = "MINCO initialization dimensions are inconsistent.";
    return false;
  }

  start_ = start;
  goal_ = goal;
  corridor_ = corridor;

  Eigen::Matrix<double, 3, 4> head_state;
  Eigen::Matrix<double, 3, 4> tail_state;
  head_state.col(0) = start.position;
  head_state.col(1) = start.velocity;
  head_state.col(2) = start.acceleration;
  head_state.col(3) = start.jerk;
  tail_state.col(0) = goal.position;
  tail_state.col(1) = goal.velocity;
  tail_state.col(2) = goal.acceleration;
  tail_state.col(3) = goal.jerk;
  minco_.setConditions(head_state, tail_state, piece_count_);

  const int waypoint_count = piece_count_ - 1;
  Eigen::VectorXd variables(3 * waypoint_count + piece_count_);
  for (int i = 0; i < waypoint_count; ++i) {
    Eigen::Vector3d waypoint = initial_waypoints[i];
    if (!corridor[i].contains(waypoint) || !corridor[i + 1].contains(waypoint)) {
      waypoint = overlapCenter(corridor[i], corridor[i + 1]);
    }
    variables.segment<3>(3 * i) = waypoint;
  }
  const int time_offset = 3 * waypoint_count;
  for (int i = 0; i < piece_count_; ++i) {
    variables(time_offset + i) =
        std::log(std::max(config_.min_piece_time, initial_times[i]));
  }

  lbfgs::lbfgs_parameter_t parameters;
  parameters.mem_size = std::max(3, config_.lbfgs_memory);
  parameters.max_iterations = std::max(1, config_.max_iterations);
  parameters.g_epsilon = config_.gradient_epsilon;
  parameters.past = 3;
  parameters.delta = config_.relative_cost_tolerance;
  parameters.max_linesearch = 48;
  double final_cost = 0.0;
  const int status = lbfgs::lbfgs_optimize(variables,
                                           final_cost,
                                           &MincoOptimizer::objectiveCallback,
                                           nullptr,
                                           nullptr,
                                           this,
                                           parameters);

  Eigen::MatrixXd waypoint_matrix;
  Eigen::VectorXd time_vector;
  decodeVariables(variables, waypoint_matrix, time_vector);
  evaluateObjective(variables, &trajectory);

  optimized_waypoints.clear();
  for (int i = 0; i < waypoint_count; ++i) {
    optimized_waypoints.push_back(waypoint_matrix.col(i));
  }
  optimized_times.assign(time_vector.data(), time_vector.data() + time_vector.size());

  std::string validation_error;
  if (!validate(trajectory, corridor, &validation_error)) {
    if (failure_reason) {
      std::ostringstream stream;
      stream << "MINCO/L-BFGS result is invalid: " << validation_error
             << " (solver status " << status << ": "
             << lbfgs::lbfgs_strerror(status) << ")";
      *failure_reason = stream.str();
    }
    return false;
  }
  return true;
}

}  // namespace bubble_planner
