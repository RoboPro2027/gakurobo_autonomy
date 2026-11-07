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
#include "gakurobo_autonomy/bt_nodes/execute_motion.hpp"

#include <chrono>
#include <sstream>
#include <string>

namespace gakurobo_autonomy
{

ExecuteMotionNode::ExecuteMotionNode(
  const std::string & name,
  const BT::NodeConfiguration & config,
  const rclcpp::Node::SharedPtr & node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
  action_client_ = rclcpp_action::create_client<ExecuteMotion>(node_, "execute_motion");
}

BT::PortsList ExecuteMotionNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("primitive", "Motion primitive name"),
    BT::InputPort<std::string>(
      "params", std::string(), "Parameters as key=value;key=value")
  };
}

BT::NodeStatus ExecuteMotionNode::onStart()
{
  const auto primitive = getInput<std::string>("primitive");
  const auto params = getInput<std::string>("params");
  if (!primitive) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to read primitive: %s", primitive.error().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!params) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to read params: %s", params.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  ExecuteMotion::Goal goal;
  goal.primitive = primitive.value();
  std::stringstream stream(params.value());
  std::string item;
  while (std::getline(stream, item, ';')) {
    const auto separator = item.find('=');
    if (separator == std::string::npos) {
      RCLCPP_ERROR(node_->get_logger(), "Invalid motion parameter: %s", item.c_str());
      return BT::NodeStatus::FAILURE;
    }
    goal.keys.push_back(item.substr(0, separator));
    goal.values.push_back(item.substr(separator + 1));
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(60))) {
    RCLCPP_ERROR(node_->get_logger(), "ExecuteMotion action server is not available.");
    return BT::NodeStatus::FAILURE;
  }

  primitive_ = goal.primitive;
  RCLCPP_INFO(node_->get_logger(), "Sending motion primitive: %s", primitive_.c_str());
  goal_handle_future_ = action_client_->async_send_goal(goal);
  state_ = State::WAITING_FOR_GOAL_RESPONSE;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecuteMotionNode::onRunning()
{
  using namespace std::chrono_literals;

  if (state_ == State::WAITING_FOR_GOAL_RESPONSE) {
    if (goal_handle_future_.wait_for(0s) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(node_->get_logger(), "Motion primitive was rejected: %s", primitive_.c_str());
      state_ = State::IDLE;
      return BT::NodeStatus::FAILURE;
    }
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
    if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
      wrapped_result.result->success)
    {
      RCLCPP_INFO(node_->get_logger(), "Motion primitive finished: %s", primitive_.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_ERROR(
      node_->get_logger(), "Motion primitive failed: %s (%s)", primitive_.c_str(),
      wrapped_result.result ? wrapped_result.result->failure_reason.c_str() : "no result");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_ERROR(node_->get_logger(), "ExecuteMotion entered an invalid internal state.");
  return BT::NodeStatus::FAILURE;
}

void ExecuteMotionNode::onHalted()
{
  if (goal_handle_) {
    RCLCPP_WARN(node_->get_logger(), "ExecuteMotion halted. Canceling active goal.");
    action_client_->async_cancel_goal(goal_handle_);
  }
  goal_handle_.reset();
  state_ = State::IDLE;
}

}  // namespace gakurobo_autonomy
