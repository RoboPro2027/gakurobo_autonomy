"""Run one code block in a child process and serve its requests (no ROS in this file).

``robot`` is whatever answers the block: the ROS adapter in primitives/run_code.py, or a fake in a
test. It has now(), pose(), command(), holding(), nav_start(), nav_state(), nav_cancel() and
log(message). The session adds cancel and the time limit, and turns every way the block can end
into one reason code the Mission can read:

    CANCELLED                     the Mission cancelled the move
    CODE_TIMEOUT                  the block ran longer than ``timeout_s`` (wall clock); it was killed
    CODE_ERROR:<line>:<type>:<message>   the block raised
    CODE_CRASHED:<exit code>      the process ended without an answer (killed, out of memory)
    <REASON>                      the block returned ("fail", "REASON")
"""

from __future__ import annotations

import json
import os
import queue
import subprocess
import sys
import threading
import time
from pathlib import Path

RUNNER = Path(__file__).with_name("code_runner.py")
STDERR_TAIL = 2000
POLL_S = 0.02


class CodeFailure(Exception):
    """The block did not succeed. The message is the reason code."""


def _reader(stream, lines):
    for line in iter(stream.readline, ""):
        lines.put(line)
    lines.put(None)


def _python():
    """The interpreter of the code blocks: the container's virtual environment, else this one."""
    return os.environ.get("GAKUROBO_CODE_PYTHON") or sys.executable


def run_block(source, params, robot, timeout_s=600.0, cancelled=lambda: False, python=None):
    """Run ``source`` with ``params``. Returns the block's result, or raises CodeFailure."""
    process = subprocess.Popen(
        [python or _python(), "-I", str(RUNNER), str(source)],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1,
    )
    lines = queue.Queue()
    threading.Thread(target=_reader, args=(process.stdout, lines), daemon=True).start()
    errors = []
    threading.Thread(target=lambda: errors.extend(iter(process.stderr.readline, "")), daemon=True).start()
    deadline = time.monotonic() + timeout_s

    def send(message):
        process.stdin.write(json.dumps(message, separators=(",", ":"), default=str) + "\n")
        process.stdin.flush()

    def stop():
        if process.poll() is None:
            process.kill()
        process.wait(timeout=5)

    try:
        send({"params": {key: str(value) for key, value in params.items()}})
        while True:
            if cancelled():
                stop()
                raise CodeFailure("CANCELLED")
            if time.monotonic() > deadline:
                stop()
                raise CodeFailure("CODE_TIMEOUT")
            try:
                line = lines.get(timeout=POLL_S)
            except queue.Empty:
                continue
            if line is None:
                process.wait(timeout=5)
                tail = "".join(errors)[-STDERR_TAIL:]
                robot.log(f"code block ended without an answer (exit {process.returncode}): {tail}")
                raise CodeFailure(f"CODE_CRASHED:{process.returncode}")
            message = json.loads(line)
            if message.get("done"):
                if "error" in message:
                    error = message["error"]
                    robot.log("code error: " + "".join(errors)[-STDERR_TAIL:])
                    raise CodeFailure(f"CODE_ERROR:{error.get('line', 0)}:{error.get('type')}:{error.get('message')}")
                result = message.get("result")
                if result is False:
                    raise CodeFailure("BLOCK_FALSE")
                if isinstance(result, list) and result and result[0] == "fail":
                    raise CodeFailure(str(result[1]) if len(result) > 1 else "BLOCK_FAILED")
                return result
            send(_answer(message, robot, cancelled))
    finally:
        stop()


def _answer(message, robot, cancelled):
    op, args = message.get("op"), message.get("args") or {}
    try:
        if op == "cancelled":
            value = bool(cancelled())
        elif op == "log":
            robot.log(str(args.get("message", "")))
            value = None
        elif op in ("now", "pose", "command", "holding", "nav_start", "nav_state", "nav_cancel", "intervention"):
            value = getattr(robot, op)(**args)
        else:
            return {"id": message.get("id"), "ok": False, "error": f"UNKNOWN_OP:{op}"}
        return {"id": message.get("id"), "ok": True, "value": value}
    except Exception as error:  # the block hears about it; the session goes on
        return {"id": message.get("id"), "ok": False, "error": f"{type(error).__name__}:{error}"}
