"""Wait for a duration of ROS time, so that it follows the simulation clock."""

import time

from gakurobo_motion.primitive import Primitive, PrimitiveError


class Delay(Primitive):
    """Model an operation by its duration. Parameter: duration_ms."""

    def execute(self, parameters, context):
        try:
            duration_ms = float(self.require(parameters, 'duration_ms'))
        except ValueError as error:
            raise PrimitiveError('INVALID_PARAMETER:duration_ms') from error
        if duration_ms < 0.0:
            raise PrimitiveError('INVALID_PARAMETER:duration_ms')
        clock = self.node.get_clock()
        deadline = clock.now().nanoseconds + int(duration_ms * 1e6)
        context.report_phase('waiting')
        while clock.now().nanoseconds < deadline:
            if context.is_cancelled():
                raise PrimitiveError('CANCELLED')
            time.sleep(0.005)
