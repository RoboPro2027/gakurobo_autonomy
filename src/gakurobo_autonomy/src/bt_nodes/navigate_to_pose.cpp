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
#include "gakurobo_autonomy/bt_nodes/navigate_to_pose.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <string>

namespace gakurobo_autonomy
{

NavigateToPoseNode::NavigateToPoseNode(
  const std::string & name,
  const BT::NodeConfiguration & config,
  const rclcpp::Node::SharedPtr & node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
  action_client_ = rclcpp_action::create_client<NavigateToPose>(
    node_, "/navigate_to_pose");
}

BT::PortsList NavigateToPoseNode::providedPorts()
{
  return {
    BT::InputPort<double>(
      "x", "Goal position x in the selected frame"),
    BT::InputPort<double>(
      "y", "Goal position y in the selected frame"),
    BT::InputPort<double>(
      "yaw", 0.0, "Goal orientation yaw in radians"),
    BT::InputPort<std::string>(
      "frame_id", std::string("odom"), "Goal coordinate frame")
  };
}

BT::NodeStatus NavigateToPoseNode::onStart()
{
  const auto x = getInput<double>("x");
  const auto y = getInput<double>("y");
  const auto yaw = getInput<double>("yaw");
  const auto frame_id = getInput<std::string>("frame_id");

  if (!x) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to read x: %s", x.error().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!y) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to read y: %s", y.error().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!yaw) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to read yaw: %s", yaw.error().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!frame_id) {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to read frame_id: %s", frame_id.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToPose action server is not available.");
    return BT::NodeStatus::FAILURE;
  }

  NavigateToPose::Goal goal;
  goal.pose.header.stamp = node_->now();
  goal.pose.header.frame_id = frame_id.value();
  goal.pose.pose.position.x = x.value();
  goal.pose.pose.position.y = y.value();
  goal.pose.pose.position.z = 0.0;

  const double half_yaw = yaw.value() * 0.5;
  goal.pose.pose.orientation.x = 0.0;
  goal.pose.pose.orientation.y = 0.0;
  goal.pose.pose.orientation.z = std::sin(half_yaw);
  goal.pose.pose.orientation.w = std::cos(half_yaw);

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.feedback_callback =
    [this](
    GoalHandle::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "Navigation feedback: remaining=%.3f m, recoveries=%d",
        feedback->distance_remaining,
        feedback->number_of_recoveries);
    };

  RCLCPP_INFO(
    node_->get_logger(),
    "Sending NavigateToPose goal: frame=%s x=%.3f y=%.3f yaw=%.3f",
    frame_id.value().c_str(), x.value(), y.value(), yaw.value());

  goal_handle_future_ = action_client_->async_send_goal(goal, options);
  state_ = State::WAITING_FOR_GOAL_RESPONSE;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToPoseNode::onRunning()
{
  using namespace std::chrono_literals;

  if (state_ == State::WAITING_FOR_GOAL_RESPONSE) {
    if (goal_handle_future_.wait_for(0s) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }

    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(node_->get_logger(), "NavigateToPose goal was rejected.");
      state_ = State::IDLE;
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "NavigateToPose goal was accepted.");
    result_future_ = action_client_->async_get_result(goal_handle_);
    state_ = State::WAITING_FOR_RESULT;
    return BT::NodeStatus::RUNNING;
  }

  if (state_ == State::WAITING_FOR_RESULT) {
    if (result_future_.wait_for(0s) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }

    const auto wrapped_result = result_future_.get();
    goal_handle_.reset();
    state_ = State::IDLE;

    switch (wrapped_result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node_->get_logger(), "NavigateToPose finished with SUCCEEDED.");
        return BT::NodeStatus::SUCCESS;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node_->get_logger(), "NavigateToPose finished with ABORTED.");
        return BT::NodeStatus::FAILURE;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(node_->get_logger(), "NavigateToPose finished with CANCELED.");
        return BT::NodeStatus::FAILURE;
      default:
        RCLCPP_ERROR(node_->get_logger(), "NavigateToPose returned an unknown result.");
        return BT::NodeStatus::FAILURE;
    }
  }

  RCLCPP_ERROR(node_->get_logger(), "NavigateToPose entered an invalid internal state.");
  return BT::NodeStatus::FAILURE;
}

void NavigateToPoseNode::onHalted()
{
  if (goal_handle_) {
    RCLCPP_WARN(node_->get_logger(), "NavigateToPose halted. Canceling active goal.");
    action_client_->async_cancel_goal(goal_handle_);
  }

  goal_handle_.reset();
  state_ = State::IDLE;
}

}  // namespace gakurobo_autonomy
