"""Base class for motion primitives.

To add a primitive: write one subclass and add it to a primitives YAML. The
Mission Runner and the ROS 2 Action stay unchanged, and the primitive can be
tried alone with ``ros2 action send_goal``.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable, Mapping


class PrimitiveError(Exception):
    """A primitive could not finish. The message is a stable reason code."""


@dataclass(frozen=True)
class PrimitiveContext:
    """What a running primitive may use besides its parameters."""

    is_cancelled: Callable[[], bool]
    report_phase: Callable[[str], None]


class Primitive(ABC):
    """One robot-local action that a Mission can call."""

    def __init__(self, node, robot: str):
        self.node = node
        self.robot = robot

    @abstractmethod
    def execute(self, parameters: Mapping[str, str], context: PrimitiveContext) -> None:
        """Run until done. Raise PrimitiveError to fail."""

    @staticmethod
    def require(parameters: Mapping[str, str], key: str) -> str:
        """Return one required parameter."""
        value = parameters.get(key, '').strip()
        if not value:
            raise PrimitiveError(f'MISSING_PARAMETER:{key}')
        return value
