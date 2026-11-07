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
#ifndef GAKUROBO_CONTROLLER__ARRIVAL_MATH_HPP_
#define GAKUROBO_CONTROLLER__ARRIVAL_MATH_HPP_

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace gakurobo_controller
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Twist2D
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

inline double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// The speed to command for an error. Near the target it is proportional, so the
// robot settles without ringing. Further away it is the speed that can still brake
// at the given deceleration. The two meet with the same speed and the same slope.
inline double approachSpeed(double error, double gain, double deceleration, double maximum)
{
  const double distance = std::abs(error);
  const double linear_limit = deceleration / (gain * gain);
  const double speed = distance <= linear_limit ?
    gain * distance :
    std::sqrt(2.0 * deceleration * (distance - 0.5 * linear_limit));
  return std::copysign(std::min(speed, maximum), error);
}

// The command in the robot's frame that drives it towards the target pose.
inline Twist2D commandTowards(
  const Pose2D & pose, const Pose2D & target, double speed, double yaw_rate)
{
  const double dx = target.x - pose.x;
  const double dy = target.y - pose.y;
  const double distance = std::hypot(dx, dy);
  const double scale = distance > 1e-9 ? speed / distance : 0.0;
  return Twist2D{
    scale * (std::cos(pose.yaw) * dx + std::sin(pose.yaw) * dy),
    scale * (-std::sin(pose.yaw) * dx + std::cos(pose.yaw) * dy),
    yaw_rate};
}

// The path left from the point nearest the robot.
inline double remainingLength(const std::vector<Pose2D> & path, const Pose2D & pose)
{
  if (path.size() < 2) {
    return path.empty() ? 0.0 : std::hypot(path[0].x - pose.x, path[0].y - pose.y);
  }
  std::size_t nearest = 0;
  double best = std::hypot(path[0].x - pose.x, path[0].y - pose.y);
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double distance = std::hypot(path[i].x - pose.x, path[i].y - pose.y);
    if (distance < best) {
      best = distance;
      nearest = i;
    }
  }
  double length = 0.0;
  for (std::size_t i = nearest; i + 1 < path.size(); ++i) {
    length += std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
  }
  return length;
}

// The point at a distance along the line from a pose towards another, keeping its heading.
inline Pose2D pointTowards(const Pose2D & from, const Pose2D & to, double distance)
{
  const double total = std::hypot(to.x - from.x, to.y - from.y);
  if (total <= distance || total < 1e-9) {
    return to;
  }
  const double ratio = distance / total;
  return Pose2D{from.x + ratio * (to.x - from.x), from.y + ratio * (to.y - from.y), to.yaw};
}

// Scale the translation down to the speed, keeping its direction and the turn.
inline Twist2D limitTranslation(const Twist2D & twist, double speed)
{
  const double current = std::hypot(twist.vx, twist.vy);
  if (current <= speed || current < 1e-9) {
    return twist;
  }
  const double scale = speed / current;
  return Twist2D{twist.vx * scale, twist.vy * scale, twist.wz};
}

// The turn to command. A small heading error is held here, so the sampling cannot
// swing a long body round in a narrow passage. With room to turn, the robot turns
// early, because near the goal it may no longer fit round. Otherwise the turn is left
// to the primary controller: a long body may only fit round while it also moves,
// which sampling finds and a turn on the spot does not.
inline double turnCommand(
  double error, double primary_turn, bool room_to_turn, double hold_error, double gain,
  double deceleration, double maximum)
{
  return room_to_turn || std::abs(error) <= hold_error ?
         approachSpeed(error, gain, deceleration, maximum) :
         primary_turn;
}

// Whether the robot was told to move and is not moving.
inline bool looksBlocked(
  const Twist2D & commanded, const Twist2D & actual, double command_speed, double still_speed)
{
  return std::hypot(commanded.vx, commanded.vy) >= command_speed &&
         std::hypot(actual.vx, actual.vy) < still_speed;
}

}  // namespace gakurobo_controller

#endif  // GAKUROBO_CONTROLLER__ARRIVAL_MATH_HPP_
