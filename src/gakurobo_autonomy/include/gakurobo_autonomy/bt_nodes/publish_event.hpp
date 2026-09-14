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
#ifndef GAKUROBO_AUTONOMY__BT_NODES__PUBLISH_EVENT_HPP_
#define GAKUROBO_AUTONOMY__BT_NODES__PUBLISH_EVENT_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace gakurobo_autonomy
{

class PublishEventNode : public BT::SyncActionNode
{
public:
  PublishEventNode(
    const std::string & name,
    const BT::NodeConfiguration & config,
    const rclcpp::Node::SharedPtr & node);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  static std::string escape_json(const std::string & value);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

}  // namespace gakurobo_autonomy

#endif  // GAKUROBO_AUTONOMY__BT_NODES__PUBLISH_EVENT_HPP_
