// Copyright 2026 RoboPro2027
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "gakurobo_controller/arrival_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace gakurobo_controller
{

namespace
{
constexpr double TRACK_SPACING_M = 0.05;
constexpr double TRACK_LENGTH_M = 3.0;
constexpr double ESCAPE_REACHED_M = 0.03;
constexpr double COMMAND_LOOKAHEAD_S = 0.3;
constexpr double SAME_GOAL_M = 0.01;
constexpr int ESCAPE_DIRECTIONS = 16;
constexpr double ESCAPE_STEP_M = 0.1;
constexpr double REARRIVE_FACTOR = 2.0;
constexpr double HOLD_HEADING_RAD = 0.3;
constexpr double NO_PATH_WINDOW_S = 1.0;
constexpr std::size_t NO_PATH_FAILURES = 5;
constexpr double TURN_CHECK_STEP_RAD = 0.1;
constexpr double TURN_CHECK_RAD = 0.6;

Pose2D toPose2D(const geometry_msgs::msg::Pose & pose)
{
  return Pose2D{pose.position.x, pose.position.y, tf2::getYaw(pose.orientation)};
}
}  // namespace

ArrivalController::ArrivalController()
: loader_("nav2_core", "nav2_core::Controller")
{
}

double ArrivalController::parameter(const std::string & name, double default_value)
{
  auto node = node_.lock();
  nav2_util::declare_parameter_if_not_declared(
    node, name, rclcpp::ParameterValue(default_value));
  return node->get_parameter(name).as_double();
}

void ArrivalController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  costmap_ros_ = costmap_ros;

  const std::string prefix = name + ".";
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "primary_controller",
    rclcpp::ParameterValue(std::string("nav2_mppi_controller::MPPIController")));
  const auto primary_type = node->get_parameter(prefix + "primary_controller").as_string();

  gain_ = parameter(prefix + "arrival.gain", gain_);
  deceleration_ = parameter(prefix + "arrival.deceleration", deceleration_);
  max_speed_ = parameter(prefix + "arrival.max_speed", max_speed_);
  yaw_gain_ = parameter(prefix + "arrival.yaw_gain", yaw_gain_);
  yaw_deceleration_ = parameter(prefix + "arrival.yaw_deceleration", yaw_deceleration_);
  max_yaw_rate_ = parameter(prefix + "arrival.max_yaw_rate", max_yaw_rate_);
  arrival_distance_ = parameter(prefix + "arrival.distance", arrival_distance_);
  escape_distance_ = parameter(prefix + "escape.distance", escape_distance_);
  escape_speed_ = parameter(prefix + "escape.speed", escape_speed_);
  escape_timeout_s_ = parameter(prefix + "escape.timeout_s", escape_timeout_s_);
  blocked_hold_s_ = parameter(prefix + "escape.blocked_hold_s", blocked_hold_s_);
  blocked_command_speed_ = parameter(prefix + "escape.blocked_command_speed", blocked_command_speed_);
  still_speed_ = parameter(prefix + "escape.still_speed", still_speed_);
  min_command_speed_ = parameter(prefix + "min_command_speed", min_command_speed_);

  primary_ = loader_.createUniqueInstance(primary_type);
  primary_->configure(parent, name, tf, costmap_ros);
  events_ = node->create_publisher<std_msgs::msg::String>("controller_events", 10);
  blocked_since_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  escape_deadline_ = blocked_since_;
}

void ArrivalController::cleanup()
{
  primary_->cleanup();
  primary_.reset();
  events_.reset();
}

void ArrivalController::activate()
{
  primary_->activate();
  events_->on_activate();
}

void ArrivalController::deactivate()
{
  primary_->deactivate();
  events_->on_deactivate();
}

void ArrivalController::setPlan(const nav_msgs::msg::Path & path)
{
  primary_->setPlan(path);
  if (path.poses.empty()) {
    return;
  }
  path_.clear();
  path_.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    path_.push_back(toPose2D(pose.pose));
  }
  const Pose2D goal = path_.back();
  if (std::hypot(goal.x - goal_.x, goal.y - goal_.y) > SAME_GOAL_M ||
    std::abs(wrapAngle(goal.yaw - goal_.yaw)) > SAME_GOAL_M)
  {
    mode_ = mode_ == Mode::ESCAPE ? mode_ : Mode::FOLLOW;
    escapes_ = 0;
  }
  goal_ = goal;
}

bool ArrivalController::isFree(const Pose2D & pose)
{
  auto * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
  collision_checker_.setCostmap(costmap);
  // An inflated cell is not a collision: only the outline on an obstacle is.
  return collision_checker_.footprintCostAtPose(
    pose.x, pose.y, pose.yaw, costmap_ros_->getRobotFootprint()) <
         nav2_costmap_2d::LETHAL_OBSTACLE;
}

void ArrivalController::remember(const Pose2D & robot)
{
  if (track_.empty() ||
    std::hypot(track_.back().x - robot.x, track_.back().y - robot.y) >= TRACK_SPACING_M)
  {
    track_.push_back(robot);
    if (track_.size() > static_cast<std::size_t>(TRACK_LENGTH_M / TRACK_SPACING_M)) {
      track_.pop_front();
    }
  }
}

bool ArrivalController::isBlocked(const Twist2D & actual)
{
  const auto now = clock_->now();
  if (!looksBlocked(last_command_, actual, blocked_command_speed_, still_speed_)) {
    blocked_since_ = rclcpp::Time(0, 0, clock_->get_clock_type());
    return false;
  }
  if (blocked_since_.nanoseconds() == 0) {
    blocked_since_ = now;
  }
  return (now - blocked_since_).seconds() >= blocked_hold_s_;
}

std::optional<Pose2D> ArrivalController::escapeTarget(const Pose2D & robot)
{
  // Where the robot came from is known to be free, if the outline still fits there.
  double travelled = 0.0;
  Pose2D previous = robot;
  for (auto it = track_.rbegin(); it != track_.rend(); ++it) {
    const double step = std::hypot(it->x - previous.x, it->y - previous.y);
    if (travelled + step >= escape_distance_) {
      const Pose2D target = pointTowards(previous, *it, escape_distance_ - travelled);
      if (isFree(target)) {
        return target;
      }
      break;
    }
    travelled += step;
    previous = *it;
  }
  // No track, as at the start of a Run: take the nearest free spot in any direction.
  for (double reach = ESCAPE_STEP_M; reach <= escape_distance_ + 1e-6; reach += ESCAPE_STEP_M) {
    for (int i = 0; i < ESCAPE_DIRECTIONS; ++i) {
      const double angle = 2.0 * M_PI * i / ESCAPE_DIRECTIONS;
      const Pose2D candidate{
        robot.x + reach * std::cos(angle), robot.y + reach * std::sin(angle), robot.yaw};
      if (isFree(candidate)) {
        return candidate;
      }
    }
  }
  return std::nullopt;
}

bool ArrivalController::startEscape(const Pose2D & robot, const std::string & reason)
{
  if (escapes_ >= max_escapes_) {
    return false;
  }
  const auto target = escapeTarget(robot);
  if (!target) {
    return false;
  }
  ++escapes_;
  escape_target_ = *target;
  escape_deadline_ = clock_->now() + rclcpp::Duration::from_seconds(escape_timeout_s_);
  mode_ = Mode::ESCAPE;
  track_.clear();
  RCLCPP_WARN(
    logger_, "Stuck (%s): escaping to (%.2f, %.2f), attempt %d", reason.c_str(),
    target->x, target->y, escapes_);
  std_msgs::msg::String event;
  event.data = reason;
  events_->publish(event);
  return true;
}

Twist2D ArrivalController::escape(const Pose2D & robot)
{
  const double distance = std::hypot(escape_target_.x - robot.x, escape_target_.y - robot.y);
  if (distance <= ESCAPE_REACHED_M || clock_->now() > escape_deadline_) {
    mode_ = Mode::FOLLOW;
    primary_->reset();
    return Twist2D{};
  }
  const double speed = std::min(escape_speed_, gain_ * distance);
  const double turn = approachSpeed(
    wrapAngle(escape_target_.yaw - robot.yaw), yaw_gain_, yaw_deceleration_, max_yaw_rate_);
  return commandTowards(robot, escape_target_, speed, turn);
}

bool ArrivalController::turnIsFree(const Pose2D & robot, double error)
{
  const double direction = error >= 0.0 ? 1.0 : -1.0;
  for (double angle = TURN_CHECK_STEP_RAD; angle <= TURN_CHECK_RAD; angle += TURN_CHECK_STEP_RAD) {
    if (!isFree(Pose2D{robot.x, robot.y, robot.yaw + direction * angle})) {
      return false;
    }
  }
  return true;
}

Twist2D ArrivalController::arrive(const Pose2D & robot)
{
  const double distance = std::hypot(goal_.x - robot.x, goal_.y - robot.y);
  const double speed = std::abs(approachSpeed(distance, gain_, deceleration_, max_speed_));
  const double turn = approachSpeed(
    wrapAngle(goal_.yaw - robot.yaw), yaw_gain_, yaw_deceleration_, max_yaw_rate_);
  const Twist2D command = commandTowards(robot, goal_, speed, 0.0);

  // Where the command takes the robot over `reach` metres (never past the goal).
  const double speed_now = std::hypot(command.vx, command.vy);
  const double reach = lookaheadDistance(speed_now, COMMAND_LOOKAHEAD_S, distance);
  const auto after = [&](double travel) {
      const double ratio = speed_now > 1e-9 ? travel / speed_now : 0.0;
      return Pose2D{
        robot.x + ratio * (std::cos(robot.yaw) * command.vx - std::sin(robot.yaw) * command.vy),
        robot.y + ratio * (std::sin(robot.yaw) * command.vx + std::cos(robot.yaw) * command.vy),
        robot.yaw};
    };
  if (isFree(robot) && !isFree(after(reach))) {
    // Something is in the way ahead. Do not stop dead (nothing then moves it: no escape, no progress):
    // creep with the largest step that is still free, and stop only where no step is.
    for (const double fraction : {0.5, 0.25, 0.1}) {
      if (isFree(after(reach * fraction))) {
        return Twist2D{command.vx * fraction, command.vy * fraction, turn};
      }
    }
    return Twist2D{0.0, 0.0, turn};
  }
  return Twist2D{command.vx, command.vy, turn};
}

Twist2D ArrivalController::follow(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker, const Pose2D & robot, double remaining)
{
  const auto cmd = primary_->computeVelocityCommands(pose, velocity, goal_checker);
  const double error = wrapAngle(goal_.yaw - robot.yaw);
  const bool room_to_turn = remaining > arrival_distance_ && turnIsFree(robot, error);
  const double turn = turnCommand(
    error, cmd.twist.angular.z, room_to_turn, HOLD_HEADING_RAD,
    yaw_gain_, yaw_deceleration_, max_yaw_rate_);
  const Twist2D twist{cmd.twist.linear.x, cmd.twist.linear.y, turn};
  return limitTranslation(
    twist, std::abs(approachSpeed(remaining, gain_, deceleration_, max_speed_)));
}

geometry_msgs::msg::TwistStamped ArrivalController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  const Pose2D robot = toPose2D(pose.pose);
  const Twist2D actual{velocity.linear.x, velocity.linear.y, velocity.angular.z};
  Twist2D command;

  if (mode_ == Mode::ESCAPE) {
    command = escape(robot);
  } else {
    remember(robot);
    const double remaining = remainingLength(path_, robot);
    if (mode_ == Mode::FOLLOW && remaining <= arrival_distance_ &&
      std::abs(wrapAngle(goal_.yaw - robot.yaw)) <= HOLD_HEADING_RAD)
    {
      mode_ = Mode::ARRIVE;
    } else if (mode_ == Mode::ARRIVE && remaining > REARRIVE_FACTOR * arrival_distance_) {
      // Pushed well away: the straight line to the goal may no longer be clear.
      mode_ = Mode::FOLLOW;
      primary_->reset();
    }
    if (isBlocked(actual) && startEscape(robot, "blocked")) {
      command = escape(robot);
    } else if (mode_ == Mode::ARRIVE) {
      command = arrive(robot);
    } else {
      try {
        command = follow(pose, velocity, goal_checker, robot, remaining);
      } catch (const nav2_core::ControllerException & error) {
        // At speed the samples may all fail for a moment. Braking usually clears it,
        // so only repeated failures count, and they need not be in a row.
        const auto now = clock_->now();
        no_path_times_.push_back(now);
        while ((now - no_path_times_.front()).seconds() > NO_PATH_WINDOW_S) {
          no_path_times_.pop_front();
        }
        primary_->reset();
        if (no_path_times_.size() >= NO_PATH_FAILURES) {
          no_path_times_.clear();
          if (!startEscape(robot, "no_path")) {
            throw;
          }
          command = escape(robot);
        }
      }
    }
  }

  command = raiseToMinimum(command, min_command_speed_);
  last_command_ = command;
  geometry_msgs::msg::TwistStamped output;
  output.header.stamp = clock_->now();
  output.header.frame_id = costmap_ros_->getBaseFrameID();
  output.twist.linear.x = command.vx;
  output.twist.linear.y = command.vy;
  output.twist.angular.z = command.wz;
  return output;
}

bool ArrivalController::cancel()
{
  return primary_->cancel();
}

void ArrivalController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  primary_->setSpeedLimit(speed_limit, percentage);
}

void ArrivalController::reset()
{
  primary_->reset();
  mode_ = Mode::FOLLOW;
  last_command_ = Twist2D{};
  escapes_ = 0;
}

}  // namespace gakurobo_controller

PLUGINLIB_EXPORT_CLASS(gakurobo_controller::ArrivalController, nav2_core::Controller)
