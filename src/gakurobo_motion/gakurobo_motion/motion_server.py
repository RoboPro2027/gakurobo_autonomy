"""Run motion primitives behind one ROS 2 Action of one robot."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from gakurobo_motion.primitive import PrimitiveContext, PrimitiveError
from gakurobo_motion.registry import load_primitives
from gakurobo_motion_interfaces.action import ExecuteMotion
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class MotionServer(Node):
    """Serve <namespace>/execute_motion. One goal runs at a time."""

    def __init__(self):
        super().__init__('motion_server')
        self.declare_parameter('robot', '')
        self.declare_parameter('primitive_files', [''])
        robot = self.get_parameter('robot').value or self.get_namespace().strip('/')
        if not robot:
            raise ValueError('Set the robot parameter or run in a robot namespace')
        share = Path(get_package_share_directory('gakurobo_motion'))
        extra = [
            path for path in self.get_parameter('primitive_files').value if path
        ]
        self.primitives = load_primitives(
            [share / 'config/primitives.yaml', *extra], self, robot
        )
        self._busy = False
        self._server = ActionServer(
            self,
            ExecuteMotion,
            'execute_motion',
            execute_callback=self._execute,
            goal_callback=self._accept_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(
            f'{robot}: primitives {sorted(self.primitives)}'
        )

    def _accept_goal(self, goal):
        if self._busy or goal.primitive not in self.primitives:
            self.get_logger().error(f'Rejected primitive: {goal.primitive}')
            return GoalResponse.REJECT
        if len(goal.keys) != len(goal.values):
            self.get_logger().error('keys and values must have the same length')
            return GoalResponse.REJECT
        self._busy = True
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        result = ExecuteMotion.Result()

        def report_phase(phase):
            goal_handle.publish_feedback(ExecuteMotion.Feedback(phase=phase))

        context = PrimitiveContext(
            is_cancelled=lambda: goal_handle.is_cancel_requested,
            report_phase=report_phase,
        )
        try:
            self.primitives[goal.primitive].execute(
                dict(zip(goal.keys, goal.values)), context
            )
            result.success = True
            goal_handle.succeed()
        except PrimitiveError as error:
            result.failure_reason = str(error)
            self.get_logger().error(f'{goal.primitive} failed: {error}')
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
        except Exception as error:  # 不具合でもサーバーを止めず、失敗として返す
            result.failure_reason = f'INTERNAL_ERROR:{error}'
            self.get_logger().error(f'{goal.primitive} crashed: {error}')
            goal_handle.abort()
        finally:
            self._busy = False
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MotionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
