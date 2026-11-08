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
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "gakurobo_controller/arrival_math.hpp"

using gakurobo_controller::approachSpeed;
using gakurobo_controller::commandTowards;
using gakurobo_controller::limitTranslation;
using gakurobo_controller::looksBlocked;
using gakurobo_controller::raiseToMinimum;
using gakurobo_controller::Pose2D;
using gakurobo_controller::pointTowards;
using gakurobo_controller::remainingLength;
using gakurobo_controller::turnCommand;
using gakurobo_controller::Twist2D;

constexpr double GAIN = 2.5;
constexpr double DECELERATION = 3.0;
constexpr double MAXIMUM = 6.0;

TEST(ApproachSpeed, IsProportionalNearTheTarget)
{
  EXPECT_NEAR(approachSpeed(0.1, GAIN, DECELERATION, MAXIMUM), 0.25, 1e-9);
}

TEST(ApproachSpeed, KeepsTheSignOfTheError)
{
  EXPECT_NEAR(approachSpeed(-0.1, GAIN, DECELERATION, MAXIMUM), -0.25, 1e-9);
}

TEST(ApproachSpeed, HasNoJumpWhereTheTwoPartsMeet)
{
  const double meeting = DECELERATION / (GAIN * GAIN);
  EXPECT_NEAR(
    approachSpeed(meeting - 1e-6, GAIN, DECELERATION, MAXIMUM),
    approachSpeed(meeting + 1e-6, GAIN, DECELERATION, MAXIMUM), 1e-4);
}

TEST(ApproachSpeed, FarAwayIsTheSpeedThatCanStillBrake)
{
  const double speed = approachSpeed(4.0, GAIN, DECELERATION, MAXIMUM);
  const double braking = speed * speed / (2.0 * DECELERATION);
  EXPECT_LT(braking, 4.0);
  EXPECT_GT(braking, 3.5);
}

TEST(ApproachSpeed, NeverExceedsTheMaximum)
{
  EXPECT_EQ(approachSpeed(100.0, GAIN, DECELERATION, MAXIMUM), MAXIMUM);
}

TEST(CommandTowards, IsInTheRobotsFrame)
{
  const auto twist = commandTowards(Pose2D{0.0, 0.0, M_PI / 2}, Pose2D{0.0, 1.0, 0.0}, 0.5, 0.0);
  EXPECT_NEAR(twist.vx, 0.5, 1e-9);
  EXPECT_NEAR(twist.vy, 0.0, 1e-9);
}

TEST(CommandTowards, MovesSidewaysForATargetBeside)
{
  const auto twist = commandTowards(Pose2D{0.0, 0.0, 0.0}, Pose2D{0.0, 1.0, 0.0}, 0.5, 0.2);
  EXPECT_NEAR(twist.vy, 0.5, 1e-9);
  EXPECT_NEAR(twist.wz, 0.2, 1e-9);
}

TEST(CommandTowards, StaysStillOnTheTarget)
{
  const auto twist = commandTowards(Pose2D{1.0, 1.0, 0.0}, Pose2D{1.0, 1.0, 0.0}, 0.5, 0.0);
  EXPECT_EQ(twist.vx, 0.0);
  EXPECT_EQ(twist.vy, 0.0);
}

TEST(RemainingLength, CountsFromTheNearestPoint)
{
  const std::vector<Pose2D> path{{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {2, 1, 0}};
  EXPECT_NEAR(remainingLength(path, Pose2D{1.1, 0.1, 0}), 2.0, 1e-9);
}

TEST(RemainingLength, OfOnePointIsTheDistanceToIt)
{
  EXPECT_NEAR(remainingLength({Pose2D{3, 4, 0}}, Pose2D{0, 0, 0}), 5.0, 1e-9);
}

TEST(PointTowards, StopsAtTheDistance)
{
  const auto point = pointTowards(Pose2D{0, 0, 0}, Pose2D{2, 0, 1}, 0.5);
  EXPECT_NEAR(point.x, 0.5, 1e-9);
  EXPECT_EQ(point.yaw, 1);
}

TEST(PointTowards, GivesTheTargetWhenItIsNearer)
{
  EXPECT_EQ(pointTowards(Pose2D{0, 0, 0}, Pose2D{0.2, 0, 1}, 0.5).x, 0.2);
}

TEST(LimitTranslation, KeepsTheDirectionAndTheTurn)
{
  const auto twist = limitTranslation(Twist2D{3.0, 4.0, 1.5}, 1.0);
  EXPECT_NEAR(twist.vx, 0.6, 1e-9);
  EXPECT_NEAR(twist.vy, 0.8, 1e-9);
  EXPECT_EQ(twist.wz, 1.5);
}

TEST(LimitTranslation, LeavesASlowCommandAlone)
{
  EXPECT_EQ(limitTranslation(Twist2D{0.3, 0.0, 0.0}, 1.0).vx, 0.3);
}

TEST(TurnCommand, HoldsASmallErrorItself)
{
  EXPECT_NEAR(turnCommand(0.1, -1.5, false, 0.3, GAIN, DECELERATION, MAXIMUM), 0.25, 1e-9);
}

TEST(TurnCommand, IgnoresASwingOfThePrimaryWhenAlreadyAligned)
{
  EXPECT_EQ(turnCommand(0.0, 1.5, false, 0.3, GAIN, DECELERATION, MAXIMUM), 0.0);
}

TEST(TurnCommand, TurnsEarlyWithRoomToTurn)
{
  EXPECT_GT(turnCommand(1.5, -0.7, true, 0.3, GAIN, DECELERATION, MAXIMUM), 0.0);
}

TEST(TurnCommand, LeavesALargeErrorToThePrimaryWithoutRoom)
{
  EXPECT_EQ(turnCommand(1.5, -0.7, false, 0.3, GAIN, DECELERATION, MAXIMUM), -0.7);
}

TEST(LooksBlocked, MeansToldToMoveAndNotMoving)
{
  EXPECT_TRUE(looksBlocked(Twist2D{0.5, 0.0, 0.0}, Twist2D{0.0, 0.0, 0.0}, 0.2, 0.03));
}

TEST(LooksBlocked, IsFalseWhenTheRobotFollows)
{
  EXPECT_FALSE(looksBlocked(Twist2D{0.5, 0.0, 0.0}, Twist2D{0.4, 0.0, 0.0}, 0.2, 0.03));
}

TEST(LooksBlocked, IsFalseWhenNotToldToMove)
{
  EXPECT_FALSE(looksBlocked(Twist2D{0.0, 0.0, 0.5}, Twist2D{0.0, 0.0, 0.0}, 0.2, 0.03));
}

TEST(RaiseToMinimum, RaisesACreepToTheMinimumInTheSameDirection)
{
  const auto out = raiseToMinimum(Twist2D{0.003, -0.004, 0.2}, 0.025);
  EXPECT_NEAR(std::hypot(out.vx, out.vy), 0.025, 1e-12);
  EXPECT_NEAR(out.vx / out.vy, 0.003 / -0.004, 1e-12);
  EXPECT_DOUBLE_EQ(out.wz, 0.2);
}

TEST(RaiseToMinimum, KeepsAStopAndATurnOnTheSpot)
{
  const auto out = raiseToMinimum(Twist2D{0.0, 0.0, 0.5}, 0.025);
  EXPECT_DOUBLE_EQ(out.vx, 0.0);
  EXPECT_DOUBLE_EQ(out.vy, 0.0);
  EXPECT_DOUBLE_EQ(out.wz, 0.5);
}

TEST(RaiseToMinimum, LeavesAFasterCommandAlone)
{
  const auto out = raiseToMinimum(Twist2D{0.3, 0.1, 0.0}, 0.025);
  EXPECT_DOUBLE_EQ(out.vx, 0.3);
  EXPECT_DOUBLE_EQ(out.vy, 0.1);
}

TEST(RaiseToMinimum, DoesNothingWithoutAMinimum)
{
  const auto out = raiseToMinimum(Twist2D{0.003, 0.0, 0.0}, 0.0);
  EXPECT_DOUBLE_EQ(out.vx, 0.003);
}
