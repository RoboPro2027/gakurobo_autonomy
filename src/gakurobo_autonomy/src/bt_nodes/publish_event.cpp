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
#include "gakurobo_autonomy/bt_nodes/publish_event.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>

namespace gakurobo_autonomy
{

PublishEventNode::PublishEventNode(
  const std::string & name,
  const BT::NodeConfiguration & config,
  const rclcpp::Node::SharedPtr & node)
: BT::SyncActionNode(name, config), node_(node)
{
  publisher_ = node_->create_publisher<std_msgs::msg::String>(
    "/experiment/events",
    rclcpp::QoS(10)
    .reliable()
    .transient_local());
}

BT::PortsList PublishEventNode::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "event_type", "Required structured event type"),
    BT::InputPort<std::string>(
      "segment_id", std::string(""), "Mission segment identifier"),
    BT::InputPort<std::string>(
      "goal_id", std::string(""), "Mission goal identifier"),
    BT::InputPort<std::string>(
      "status", std::string(""), "Event status")
  };
}

std::string PublishEventNode::escape_json(const std::string & value)
{
  std::ostringstream escaped;
  for (const unsigned char character : value) {
    switch (character) {
      case '"': escaped << "\\\""; break;
      case '\\': escaped << "\\\\"; break;
      case '\b': escaped << "\\b"; break;
      case '\f': escaped << "\\f"; break;
      case '\n': escaped << "\\n"; break;
      case '\r': escaped << "\\r"; break;
      case '\t': escaped << "\\t"; break;
      default:
        if (character < 0x20U) {
          escaped << "\\u" << std::hex << std::setw(4)
                  << std::setfill('0') << static_cast<int>(character)
                  << std::dec << std::setfill(' ');
        } else {
          escaped << static_cast<char>(character);
        }
    }
  }
  return escaped.str();
}

BT::NodeStatus PublishEventNode::tick()
{
  const auto event_type = getInput<std::string>("event_type");
  if (!event_type || event_type.value().empty()) {
    RCLCPP_ERROR(
      node_->get_logger(), "PublishEvent requires a non-empty event_type.");
    return BT::NodeStatus::FAILURE;
  }

  const auto segment_id = getInput<std::string>("segment_id").value_or("");
  const auto goal_id = getInput<std::string>("goal_id").value_or("");
  const auto status = getInput<std::string>("status").value_or("");

  const auto wall_time_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();

  std::ostringstream json;
  json << "{\"schema_version\":1"
       << ",\"event_type\":\"" << escape_json(event_type.value()) << "\""
       << ",\"source\":\"gakurobo_autonomy.mission_runner\""
       << ",\"wall_time_ns\":" << wall_time_ns;
  if (!segment_id.empty()) {
    json << ",\"segment_id\":\"" << escape_json(segment_id) << "\"";
  }
  if (!goal_id.empty()) {
    json << ",\"goal_id\":\"" << escape_json(goal_id) << "\"";
  }
  if (!status.empty()) {
    json << ",\"status\":\"" << escape_json(status) << "\"";
  }
  json << "}";

  constexpr auto discovery_timeout =
    std::chrono::milliseconds(1000);
  constexpr auto discovery_poll =
    std::chrono::milliseconds(20);

  const auto discovery_deadline =
    std::chrono::steady_clock::now() +
    discovery_timeout;

  while (
    rclcpp::ok() &&
    publisher_->get_subscription_count() == 0U &&
    std::chrono::steady_clock::now() <
    discovery_deadline)
  {
    rclcpp::sleep_for(discovery_poll);
  }

  const auto subscription_count =
    publisher_->get_subscription_count();

  if (subscription_count == 0U) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Publishing experiment event without subscribers: %s",
      event_type.value().c_str());
  }

  std_msgs::msg::String message;
  message.data = json.str();
  publisher_->publish(message);

  RCLCPP_INFO(
    node_->get_logger(),
    "Published experiment event: %s subscribers=%zu",
    event_type.value().c_str(),
    subscription_count);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace gakurobo_autonomy
