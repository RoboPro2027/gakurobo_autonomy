"""A real rclpy goal handle compares with ==, which reads other.goal_id: cancelling must not use ==."""
import threading

from gakurobo_motion.primitives.run_code import RosRobot


class Handle:
    """What rclpy.action.client.ClientGoalHandle does on == (it reads the other side's goal_id)."""

    def __init__(self):
        self.cancelled = 0
        self.goal_id = "id"

    def __eq__(self, other):
        return self.goal_id == other.goal_id

    def cancel_goal_async(self):
        self.cancelled += 1


def robot_with(goal):
    robot = object.__new__(RosRobot)
    robot._lock = threading.Lock()
    robot._goal, robot._result = goal, None
    return robot


def test_cancelling_a_real_handle_asks_nav2_to_cancel():
    handle = Handle()
    robot = robot_with(handle)
    robot.nav_cancel()
    assert handle.cancelled == 1 and robot._goal is None and robot._result == "canceled"


def test_cancelling_with_no_goal_or_a_goal_still_being_sent_is_quiet():
    for goal in (None, "sending"):
        robot = robot_with(goal)
        robot.nav_cancel()
        assert robot._goal is None
