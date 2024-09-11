import numpy
import msgpack
import msgpack_numpy

from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Any

msgpack_numpy.patch()


class FunctionResult(IntEnum):
    SUCCESS = 0
    FAIL = -1


class ControlMode(IntEnum):
    NONE = 0x00
    CURRENT = 0x01
    EFFORT = 0x02
    VELOCITY = 0x03
    POSITION = 0x04
    PD = 0x09
    OTHER = 0x0A


class ControlConfig:
    control_mode: ControlMode
    position_control_kp = None
    velocity_control_kp = None
    velocity_control_ki = None
    pd_control_kp = None
    pd_control_kd = None

    def to_dict(self):
        return {
            "control_mode": self.control_mode,
            "position_control_kp": self.position_control_kp,
            "velocity_control_kp": self.velocity_control_kp,
            "velocity_control_ki": self.velocity_control_ki,
            "pd_control_kp": self.pd_control_kp,
            "pd_control_kd": self.pd_control_kd,
        }


class ControlGroup(tuple, Enum):
    ALL = (0, 32)
    LEFT_LEG = (0, 6)
    RIGHT_LEG = (6, 6)
    WAIST = (12, 3)
    HEAD = (15, 3)
    LEFT_ARM = (18, 7)
    RIGHT_ARM = (25, 7)
    LOWER = (0, 18)
    UPPER = (18, 14)
    UPPER_EXTENDED = (12, 20)

    @property
    def slice(self):
        return slice(self.value[0], self.value[0] + self.value[1])

    @property
    def num_joints(self):
        return self.value[1]


class Serde:
    @staticmethod
    def pack(data: Any) -> bytes:
        return msgpack.packb(data)  # type: ignore

    @staticmethod
    def unpack(data: bytes):
        return msgpack.unpackb(data)


class ServiceStatus(str, Enum):
    OK = "OK"
    ERROR = "ERROR"


@dataclass
class Trajectory:
    start: numpy.ndarray
    end: numpy.ndarray
    duration: float

    def at(self, t: float):
        return self.start + (self.end - self.start) * t / self.duration

    def finished(self, t: float):
        return t >= self.duration
