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
#ifndef GAKUROBO_AUTONOMY__TREE_STATUS_PUBLISHER_HPP_
#define GAKUROBO_AUTONOMY__TREE_STATUS_PUBLISHER_HPP_

#include <chrono>
#include <memory>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gakurobo_autonomy
{

// Publish every node status change on <namespace>/tree_status.
// key is the node name and value is RUNNING, SUCCESS or FAILURE. The return to
// IDLE is not published, so a viewer keeps the result of the finished nodes.
class TreeStatusPublisher : public BT::StatusChangeLogger
{
public:
  TreeStatusPublisher(const rclcpp::Node::SharedPtr & node, const BT::Tree & tree)
  : BT::StatusChangeLogger(tree.rootNode()),
    publisher_(node->create_publisher<diagnostic_msgs::msg::KeyValue>(
        "tree_status", rclcpp::QoS(100).reliable()))
  {
    enableTransitionToIdle(false);
  }

  void callback(
    BT::Duration,
    const BT::TreeNode & node,
    BT::NodeStatus,
    BT::NodeStatus status) override
  {
    diagnostic_msgs::msg::KeyValue message;
    message.key = node.name();
    message.value = BT::toStr(status, false);
    publisher_->publish(message);
  }

  // Wait for the subscribers, so the last status is not lost when the runner exits.
  void flush() override
  {
    publisher_->wait_for_all_acked(std::chrono::milliseconds(500));
  }

private:
  rclcpp::Publisher<diagnostic_msgs::msg::KeyValue>::SharedPtr publisher_;
};

}  // namespace gakurobo_autonomy

#endif  // GAKUROBO_AUTONOMY__TREE_STATUS_PUBLISHER_HPP_
