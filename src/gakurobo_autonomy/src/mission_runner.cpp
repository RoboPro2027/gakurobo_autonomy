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
#include <utility>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "gakurobo_autonomy/bt_nodes/execute_motion.hpp"
#include "gakurobo_autonomy/bt_nodes/navigate_to_pose.hpp"
#include "gakurobo_autonomy/bt_nodes/publish_event.hpp"
#include "gakurobo_autonomy/tree_status_publisher.hpp"

class LogMessageNode : public BT::SyncActionNode
{
public:
  LogMessageNode(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("message", "Message written to the ROS log")
    };
  }

  BT::NodeStatus tick() override
  {
    const auto message = getInput<std::string>("message");
    if (!message) {
      throw BT::RuntimeError("LogMessage requires input port: ", message.error());
    }

    RCLCPP_INFO(
      rclcpp::get_logger("mission_runner"), "%s", message.value().c_str());
    return BT::NodeStatus::SUCCESS;
  }
};

class WaitNode : public BT::StatefulActionNode
{
public:
  WaitNode(
    const std::string & name,
    const BT::NodeConfiguration & config,
    rclcpp::Clock::SharedPtr clock)
  : BT::StatefulActionNode(name, config),
    clock_(std::move(clock))
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<unsigned int>(
        "delay_ms", 1000U, "Wait duration in milliseconds")
    };
  }

  BT::NodeStatus onStart() override
  {
    const auto delay_ms = getInput<unsigned int>("delay_ms");
    if (!delay_ms) {
      throw BT::RuntimeError("Wait requires input port: ", delay_ms.error());
    }

    deadline_ = clock_->now() +
      rclcpp::Duration(std::chrono::milliseconds(delay_ms.value()));

    RCLCPP_INFO(
      rclcpp::get_logger("mission_runner"),
      "Waiting for %u ms", delay_ms.value());
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (clock_->now() >= deadline_) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    RCLCPP_WARN(rclcpp::get_logger("mission_runner"), "Wait node halted.");
  }

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time deadline_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mission_runner");

  node->declare_parameter<std::string>("bt_xml", "");
  const auto xml_path = node->get_parameter("bt_xml").as_string();

  if (xml_path.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter [bt_xml] is empty.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Loading behavior tree: %s", xml_path.c_str());

  try {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<LogMessageNode>("LogMessage");
    factory.registerBuilder<WaitNode>(
      "Wait",
      [clock = node->get_clock()](
        const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<WaitNode>(name, config, clock);
      });

    BT::NodeBuilder navigate_builder =
      [node](
      const std::string & name,
      const BT::NodeConfiguration & config)
      {
        return std::make_unique<gakurobo_autonomy::NavigateToPoseNode>(
          name, config, node);
      };

    factory.registerBuilder<gakurobo_autonomy::NavigateToPoseNode>(
      "NavigateToPose", navigate_builder);

    BT::NodeBuilder publish_event_builder =
      [node](
      const std::string & name,
      const BT::NodeConfiguration & config)
      {
        return std::make_unique<gakurobo_autonomy::PublishEventNode>(
          name, config, node);
      };

    factory.registerBuilder<gakurobo_autonomy::PublishEventNode>(
      "PublishEvent", publish_event_builder);

    BT::NodeBuilder execute_motion_builder =
      [node](
      const std::string & name,
      const BT::NodeConfiguration & config)
      {
        return std::make_unique<gakurobo_autonomy::ExecuteMotionNode>(
          name, config, node);
      };

    factory.registerBuilder<gakurobo_autonomy::ExecuteMotionNode>(
      "ExecuteMotion", execute_motion_builder);

    auto tree = factory.createTreeFromFile(xml_path);
    gakurobo_autonomy::TreeStatusPublisher status_publisher(node, tree);

    while (rclcpp::ok()) {
      const auto status = tree.tickRoot();
      rclcpp::spin_some(node);

      if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Mission finished with SUCCESS.");
        status_publisher.flush();
        break;
      }

      if (status == BT::NodeStatus::FAILURE) {
        RCLCPP_ERROR(node->get_logger(), "Mission finished with FAILURE.");
        status_publisher.flush();
        rclcpp::shutdown();
        return 1;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      node->get_logger(), "Behavior tree exception: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
