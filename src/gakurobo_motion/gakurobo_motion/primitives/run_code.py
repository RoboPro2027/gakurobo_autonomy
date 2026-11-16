"""Run a code block: a Python file of the Motion Library, in a child process (ADR-0022).

Parameters: ``block`` (its name), ``source`` (the file), ``sha`` (its SHA-256: a Run keeps the file it
ran, and a file that is not that one is refused), ``timeout_s`` (wall clock; default 600), and the
block's own parameters, which are passed to it as strings.

The block never touches ROS. It asks this primitive for the robot's pose, its last command, whether it
holds something, and for Nav2 goals; see code_runner.py. The robot that answers is this one: a block
cannot read the other robot.
"""

import hashlib
import math
from pathlib import Path
import threading
import time

from gakurobo_motion.code_session import CodeFailure, run_block
from gakurobo_motion.primitive import Primitive, PrimitiveError

RESERVED = {"block", "source", "sha", "timeout_s"}
SELECTOR_SETTLE_S = 0.3  # Nav2's selectors read the name they were last sent


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class RosRobot:
    """What a block may ask, answered from this node's ROS graph."""

    def __init__(self, node):
        from geometry_msgs.msg import Twist  # noqa: F401  (imported where the node runs)
        from nav2_msgs.action import NavigateToPose
        from nav_msgs.msg import Odometry
        from rclpy.action import ActionClient
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        self.node = node
        self._pose, self._command, self._holding = None, (0.0, 0.0, 0.0), False
        self._goal, self._result, self._remaining = None, None, None
        self._lock = threading.Lock()
        node.create_subscription(Odometry, "odom", self._on_odom, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self._controller = node.create_publisher(String, "controller_selector", latched)
        self._planner = node.create_publisher(String, "planner_selector", latched)
        self._client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self._NavigateToPose, self._String = NavigateToPose, String
        try:
            from geometry_msgs.msg import Twist
            node.create_subscription(Twist, "cmd_vel_nav", self._on_command, 10)
        except ImportError:
            pass
        try:
            from gakurobo_interfaces.msg import RobotObservation
            node.create_subscription(RobotObservation, "observation", self._on_observation, latched)
        except ImportError:
            pass

    def _on_odom(self, message):
        p = message.pose.pose
        with self._lock:
            self._pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_command(self, message):
        with self._lock:
            self._command = (message.linear.x, message.linear.y, message.angular.z)

    def _on_observation(self, message):
        with self._lock:
            self._holding = any(holder.occupied for holder in message.holders)

    # --- what a block may ask ---
    def now(self):
        return self.node.get_clock().now().nanoseconds / 1e9

    def pose(self):
        with self._lock:
            if self._pose is None:
                raise RuntimeError("NO_ODOMETRY")
            return list(self._pose)

    def command(self):
        with self._lock:
            return list(self._command)

    def holding(self):
        with self._lock:
            return self._holding

    def log(self, message):
        self.node.get_logger().info(f"code: {message}")

    def nav_start(self, x, y, yaw, frame_id="map", controller="", planner=""):
        self.nav_cancel()
        for publisher, name, default in ((self._controller, controller, "FollowPath"), (self._planner, planner, "GridBased")):
            message = self._String()
            message.data = name or default
            publisher.publish(message)
        time.sleep(SELECTOR_SETTLE_S)
        if not self._client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("NAV_UNAVAILABLE")
        goal = self._NavigateToPose.Goal()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        with self._lock:
            self._goal, self._result, self._remaining = "sending", None, None
        future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_feedback(self, message):
        with self._lock:
            self._remaining = float(message.feedback.distance_remaining)

    def _on_goal_response(self, future):
        handle = future.result()
        with self._lock:
            if not handle.accepted:
                self._goal, self._result = None, "failed"
                return
            self._goal = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        status = future.result().status  # 4 succeeded, 5 canceled, 6 aborted
        with self._lock:
            self._result = {4: "succeeded", 5: "canceled"}.get(status, "failed")

    def nav_state(self):
        with self._lock:
            if self._result:
                return {"state": self._result, "remaining": self._remaining}
            return {"state": "running" if self._goal else "idle", "remaining": self._remaining}

    def nav_cancel(self):
        with self._lock:
            handle, self._goal = self._goal, None
        if handle not in (None, "sending") and hasattr(handle, "cancel_goal_async"):
            handle.cancel_goal_async()
        with self._lock:
            self._result = "canceled" if handle else self._result


class RunCode(Primitive):
    def __init__(self, node, robot):
        super().__init__(node, robot)
        self._robot = None

    def _ros_robot(self):
        if self._robot is None:
            self._robot = RosRobot(self.node)
        return self._robot

    def execute(self, parameters, context):
        source = Path(self.require(parameters, "source"))
        expected = self.require(parameters, "sha")
        self.require(parameters, "block")
        try:
            actual = hashlib.sha256(source.read_bytes()).hexdigest()
        except OSError as error:
            raise PrimitiveError("CODE_NOT_FOUND") from error
        if actual != expected.removeprefix("sha256:"):
            raise PrimitiveError("CODE_HASH_MISMATCH")
        try:
            timeout_s = float(parameters.get("timeout_s", 600.0))
        except ValueError as error:
            raise PrimitiveError("INVALID_PARAMETER:timeout_s") from error
        params = {key: value for key, value in parameters.items() if key not in RESERVED}
        context.report_phase("running_code")
        robot = self._ros_robot()
        try:
            run_block(source, params, robot, timeout_s=timeout_s, cancelled=context.is_cancelled)
        except CodeFailure as failure:
            robot.nav_cancel()
            raise PrimitiveError(str(failure)) from failure
