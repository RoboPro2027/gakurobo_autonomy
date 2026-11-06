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
#ifndef GAKUROBO_AUTONOMY__BT_NODES__EXECUTE_MOTION_HPP_
#define GAKUROBO_AUTONOMY__BT_NODES__EXECUTE_MOTION_HPP_

#include <future>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "gakurobo_motion_interfaces/action/execute_motion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace gakurobo_autonomy
{

// The only node that motion primitives need. New primitives are added to the
// motion server, not here. The action name is relative to the runner namespace.
class ExecuteMotionNode : public BT::StatefulActionNode
{
public:
  using ExecuteMotion = gakurobo_motion_interfaces::action::ExecuteMotion;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteMotion>;

  ExecuteMotionNode(
    const std::string & name,
    const BT::NodeConfiguration & config,
    const rclcpp::Node::SharedPtr & node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class State
  {
    IDLE,
    WAITING_FOR_GOAL_RESPONSE,
    WAITING_FOR_RESULT
  };

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ExecuteMotion>::SharedPtr action_client_;
  State state_{State::IDLE};
  std::string primitive_;

  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  std::shared_future<GoalHandle::WrappedResult> result_future_;
  GoalHandle::SharedPtr goal_handle_;
};

}  // namespace gakurobo_autonomy

#endif  // GAKUROBO_AUTONOMY__BT_NODES__EXECUTE_MOTION_HPP_
