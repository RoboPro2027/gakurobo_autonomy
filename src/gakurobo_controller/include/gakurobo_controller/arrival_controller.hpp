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
#ifndef GAKUROBO_CONTROLLER__ARRIVAL_CONTROLLER_HPP_
#define GAKUROBO_CONTROLLER__ARRIVAL_CONTROLLER_HPP_

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "gakurobo_controller/arrival_math.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"

namespace gakurobo_controller
{

// Wraps the primary controller, which follows the path and keeps clear of obstacles.
//
// MPPI samples at random, so it cannot settle within a few centimetres, and with a
// real outline it stops for good once the outline touches an obstacle. So this
// controller takes over in two cases. Near the goal it drives there by position
// control. When the primary finds no way, or the robot is told to move and does not,
// it backs away along its track or into free space and hands control back.
//
// Every value comes from parameters, which mass computes from the Actor.
class ArrivalController : public nav2_core::Controller
{
public:
  ArrivalController();

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  bool cancel() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  void reset() override;

private:
  enum class Mode { FOLLOW, ARRIVE, ESCAPE };

  double parameter(const std::string & name, double default_value);
  bool isFree(const Pose2D & pose);
  Twist2D follow(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker, const Pose2D & robot, double remaining);
  bool turnIsFree(const Pose2D & robot, double error);
  Twist2D arrive(const Pose2D & robot);
  Twist2D escape(const Pose2D & robot);
  bool startEscape(const Pose2D & robot, const std::string & reason);
  std::optional<Pose2D> escapeTarget(const Pose2D & robot);
  void remember(const Pose2D & robot);
  bool isBlocked(const Twist2D & actual);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("ArrivalController")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> collision_checker_;
  pluginlib::ClassLoader<nav2_core::Controller> loader_;
  nav2_core::Controller::Ptr primary_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr events_;

  double gain_{2.0};
  double deceleration_{1.0};
  double max_speed_{1.0};
  double yaw_gain_{2.0};
  double yaw_deceleration_{1.0};
  double max_yaw_rate_{1.0};
  double arrival_distance_{0.3};
  double escape_distance_{0.4};
  double escape_speed_{0.3};
  double escape_timeout_s_{4.0};
  double blocked_hold_s_{0.5};
  double min_command_speed_{0.0};       // 0: no minimum. mass sets it from the Actor
  double blocked_command_speed_{0.2};   // stuck if told at least this fast ...
  double still_speed_{0.03};            // ... and moving slower than this
  int max_escapes_{4};

  Mode mode_{Mode::FOLLOW};
  std::vector<Pose2D> path_;
  Pose2D goal_;
  std::deque<Pose2D> track_;
  Pose2D escape_target_;
  rclcpp::Time escape_deadline_;
  rclcpp::Time blocked_since_;
  std::deque<rclcpp::Time> no_path_times_;
  Twist2D last_command_;
  int escapes_{0};
};

}  // namespace gakurobo_controller

#endif  // GAKUROBO_CONTROLLER__ARRIVAL_CONTROLLER_HPP_
