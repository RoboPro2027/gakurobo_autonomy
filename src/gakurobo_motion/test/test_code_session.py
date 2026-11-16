import sys
import textwrap
import time

import pytest

from gakurobo_motion.code_session import CodeFailure, run_block


class Robot:
    def __init__(self):
        self.logs, self.clock, self.goals = [], 0.0, []

    def log(self, message):
        self.logs.append(message)

    def now(self):
        self.clock += 0.1
        return self.clock

    def pose(self):
        return [1.0, 2.0, 0.5]

    def command(self):
        return [0.0, 0.0, 0.0]

    def holding(self):
        return True

    def nav_start(self, x, y, yaw, frame_id, controller, planner):
        self.goals.append((x, y, yaw, controller, planner))

    def nav_state(self):
        return {"state": "running", "remaining": 1.5}

    def nav_cancel(self):
        self.goals.append("cancel")


def block(tmp_path, body):
    path = tmp_path / "block.py"
    path.write_text(textwrap.dedent(body), encoding="utf-8")
    return path


def test_a_block_reads_the_robot_and_starts_a_goal(tmp_path):
    robot = Robot()
    path = block(tmp_path, """
        def run(ctx, params):
            x, y, yaw = ctx.pose()
            ctx.nav.start(x + ctx.number("dx", 0), y, yaw, controller="FollowPath_L1", planner="Smac")
            ctx.log("state " + ctx.nav.state()["state"] + " holding " + str(ctx.holding()))
            print("a print must not break the protocol")
            return True
    """)
    assert run_block(path, {"dx": "0.5"}, robot, timeout_s=20) is True
    assert robot.goals == [(1.5, 2.0, 0.5, "FollowPath_L1", "Smac")]
    assert "state running holding True" in robot.logs


def test_a_false_result_and_a_named_failure_are_reasons_the_mission_can_read(tmp_path):
    with pytest.raises(CodeFailure, match="BLOCK_FALSE"):
        run_block(block(tmp_path, "def run(ctx, params):\n    return False\n"), {}, Robot(), timeout_s=20)
    with pytest.raises(CodeFailure, match="STALL_UNRESOLVED"):
        run_block(block(tmp_path, "def run(ctx, params):\n    return ('fail', 'STALL_UNRESOLVED')\n"), {}, Robot(), timeout_s=20)


def test_an_error_in_the_block_comes_back_with_its_line(tmp_path):
    path = block(tmp_path, """
        def run(ctx, params):
            value = 1
            return value / 0
    """)
    with pytest.raises(CodeFailure) as error:
        run_block(path, {}, Robot(), timeout_s=20)
    assert str(error.value).startswith("CODE_ERROR:4:ZeroDivisionError:")


def test_a_block_without_run_is_a_code_error(tmp_path):
    with pytest.raises(CodeFailure, match="CODE_ERROR:0:TypeError"):
        run_block(block(tmp_path, "x = 1\n"), {}, Robot(), timeout_s=20)


def test_the_time_limit_kills_a_block_that_does_not_end(tmp_path):
    path = block(tmp_path, "def run(ctx, params):\n    while True:\n        pass\n")
    started = time.monotonic()
    with pytest.raises(CodeFailure, match="CODE_TIMEOUT"):
        run_block(path, {}, Robot(), timeout_s=1.0)
    assert time.monotonic() - started < 6


def test_a_cancel_stops_a_block_that_is_waiting(tmp_path):
    path = block(tmp_path, "def run(ctx, params):\n    ctx.sleep(1000)\n    return True\n")
    asked = {"at": None}

    def cancelled():
        asked["at"] = asked["at"] or time.monotonic()
        return time.monotonic() - asked["at"] > 0.5

    with pytest.raises(CodeFailure, match="CANCELLED"):
        run_block(path, {}, Robot(), timeout_s=30, cancelled=cancelled)


def test_a_process_that_dies_is_a_crash_not_a_hang(tmp_path):
    path = block(tmp_path, "import os\ndef run(ctx, params):\n    os._exit(7)\n")
    with pytest.raises(CodeFailure, match="CODE_CRASHED:7"):
        run_block(path, {}, Robot(), timeout_s=20)


def test_a_rate_keeps_a_loop_on_a_clock_and_counts_the_turns_that_ran_long(tmp_path):
    robot = Robot()
    slow = block(tmp_path, """
        def run(ctx, params):
            rate = ctx.rate(2)
            for _ in range(3):
                assert rate.sleep()
            rate.report()
            return True
    """)
    assert run_block(slow, {}, robot, timeout_s=20) is True
    assert any(line.startswith("rate: 3 ticks") for line in robot.logs)
    # 1 kHz is faster than the robot's clock can follow here: every turn runs long and is counted
    robot = Robot()
    fast = block(tmp_path, """
        def run(ctx, params):
            rate = ctx.rate(1000)
            for _ in range(3):
                rate.sleep()
            rate.report()
            return True
    """)
    assert run_block(fast, {}, robot, timeout_s=20) is True
    assert "rate: 3 ticks, 3 overrun(s) at 1000 Hz" in robot.logs
