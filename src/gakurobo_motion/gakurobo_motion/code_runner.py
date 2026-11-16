"""The child process of ``run_code``: it runs one code block and never imports ROS.

The parent (primitives/run_code.py) owns the robot. This process talks to it over its own
stdin and stdout, one JSON object per line, and only through the ``ctx`` it hands to the block:

    parent -> child   {"params": {...}}                       once, to start
    child  -> parent  {"id": 1, "op": "pose", "args": {}}     a request
    parent -> child   {"id": 1, "ok": true, "value": ...}     its answer
    child  -> parent  {"done": true, "result": ...}           the end, or {"done": true, "error": {...}}

A block is a file with ``run(ctx, params)``. It returns None or True for success, False or
``("fail", "REASON")`` for a failure the Mission can read. An exception is a code error: the line,
the type and the message go back to the parent, and the trace goes to stderr.

Anything the block prints goes to stderr, so it cannot break the protocol. The standard library is
all this file needs: the block's own packages come from whichever Python runs it (the virtual
environment of the container).
"""

from __future__ import annotations

import json
import runpy
import sys
import traceback


class BlockError(Exception):
    """The parent refused a request (the block may catch it, or let it end the block)."""


class Context:
    """What a block may use. Every method asks the parent."""

    def __init__(self, connection, params):
        self._connection = connection
        self.params = dict(params)
        self.nav = Navigation(self)

    def call(self, op, **args):
        return self._connection.request(op, args)

    def now(self):
        """Seconds on the robot's clock (the simulation clock in a simulation)."""
        return self.call("now")

    def pose(self):
        """(x, y, yaw) of the robot in the map."""
        return tuple(self.call("pose"))

    def command(self):
        """(vx, vy, wz) the controller last sent."""
        return tuple(self.call("command"))

    def holding(self):
        """Whether any hand holds an object."""
        return bool(self.call("holding"))

    def cancelled(self):
        return bool(self.call("cancelled"))

    def log(self, message):
        self.call("log", message=str(message))

    def sleep(self, seconds, step=0.05):
        """Wait on the robot's clock; ends early (returns False) when the Mission cancels."""
        end = self.now() + seconds
        while self.now() < end:
            if self.cancelled():
                return False
            self._connection.pause(step)
        return True

    def intervention(self, kind, start_s, end_s=None, **detail):
        """Say that this block stepped in, so a comparison can leave that span out (ADR-0023 of mass).

        ``start_s``/``end_s`` are the robot's clock: from when the move was not going (the stall began) to when it
        was going again (``end_s`` None: now). ``kind`` is what was done (``"controller"``, ``"planner"``, ``"realign"``,
        ``"retry"``...); ``detail`` is kept with it (JSON values). The Run's bag records it; nothing else depends on it.
        """
        self.call("intervention", kind=str(kind), start_s=float(start_s),
                  end_s=float(self.now() if end_s is None else end_s), detail=detail)

    def rate(self, hz):
        """A fixed-period loop on the robot's clock: ``rate = ctx.rate(10)`` then ``rate.sleep()`` each turn."""
        return Rate(self, hz)

    def number(self, key, default):
        """A parameter as a float (parameters arrive as strings)."""
        try:
            return float(self.params.get(key, default))
        except (TypeError, ValueError):
            return float(default)


class Rate:
    """Keep a loop at ``hz`` on the robot's clock (the simulation clock in a simulation).

    ``sleep()`` waits for the next tick and returns False when the Mission cancels. A turn that took
    longer than the period is counted in ``overruns`` and the clock is not caught up with a burst of
    turns: the next tick is one period after now. ``ctx.log`` reports the count when the block asks
    (``rate.report()``), so a slow block is visible in the Run's log.
    """

    def __init__(self, context, hz):
        if not hz or hz <= 0:
            raise ValueError("rate must be positive")
        self._context, self.period = context, 1.0 / float(hz)
        self._next = context.now() + self.period
        self.ticks = 0
        self.overruns = 0

    def sleep(self):
        now = self._context.now()
        self.ticks += 1
        if now > self._next + 1e-9:
            self.overruns += 1
            self._next = now + self.period
            return not self._context.cancelled()
        wait = self._next - now
        self._next += self.period
        return self._context.sleep(wait) if wait > 0 else not self._context.cancelled()

    def report(self):
        self._context.log(f"rate: {self.ticks} ticks, {self.overruns} overrun(s) at {1.0 / self.period:g} Hz")


class Navigation:
    """Send and watch Nav2 goals of this robot."""

    def __init__(self, context):
        self._context = context

    def start(self, x, y, yaw, frame_id="map", controller="", planner=""):
        """Start a goal (the earlier one is cancelled). Empty names mean the robot's own."""
        self._context.call("nav_start", x=x, y=y, yaw=yaw, frame_id=frame_id,
                           controller=controller, planner=planner)

    def state(self):
        """{"state": idle|running|succeeded|failed|canceled, "remaining": m or None}."""
        return self._context.call("nav_state")

    def cancel(self):
        self._context.call("nav_cancel")


class Connection:
    def __init__(self, reader, writer):
        self._reader, self._writer, self._next = reader, writer, 0

    def send(self, message):
        self._writer.write(json.dumps(message, separators=(",", ":")) + "\n")
        self._writer.flush()

    def request(self, op, args):
        self._next += 1
        self.send({"id": self._next, "op": op, "args": args})
        line = self._reader.readline()
        if not line:
            raise SystemExit(3)  # the parent went away
        answer = json.loads(line)
        if not answer.get("ok"):
            raise BlockError(answer.get("error") or "REFUSED")
        return answer.get("value")

    def pause(self, seconds):
        import time
        time.sleep(seconds)


def _own_frame(error, source):
    """The line of the block's file in an exception's trace, and its message."""
    line = 0
    for frame in traceback.extract_tb(error.__traceback__):
        if frame.filename == source:
            line = frame.lineno
    return {"line": line, "type": type(error).__name__, "message": str(error)[:300]}


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if len(argv) != 1:
        sys.stderr.write("usage: code_runner.py <block.py>\n")
        return 2
    source = argv[0]
    protocol_out, protocol_in = sys.stdout, sys.stdin
    sys.stdout = sys.stderr  # a print() in the block must not reach the parent as a message
    connection = Connection(protocol_in, protocol_out)
    first = json.loads(protocol_in.readline() or "{}")
    context = Context(connection, first.get("params") or {})
    try:
        namespace = runpy.run_path(source)
        run = namespace.get("run")
        if not callable(run):
            raise TypeError("the block has no run(ctx, params)")
        result = run(context, dict(context.params))
        connection.send({"done": True, "result": result if isinstance(result, (bool, type(None), list, tuple, str)) else str(result)})
        return 0
    except SystemExit:
        raise
    except BaseException as error:  # a code error is reported, never raised in the parent
        traceback.print_exc()
        connection.send({"done": True, "error": _own_frame(error, source)})
        return 1


if __name__ == "__main__":
    sys.exit(main())
