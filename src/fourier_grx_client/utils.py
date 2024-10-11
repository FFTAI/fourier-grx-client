from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Any

import msgpack
import msgpack_numpy as m
import numpy as np
from scipy.spatial.transform import Rotation as R

m.patch()


def se3_to_xyzquat(se3):
    translation = se3[:3, 3]
    rotmat = se3[:3, :3]

    quat = R.from_matrix(rotmat.copy()).as_quat()

    xyzquat = np.concatenate([translation, quat])
    return xyzquat


def xyzquat_to_se3(xyzquat):
    translation = xyzquat[:3]
    quat = xyzquat[3:]

    rotmat = R.from_quat(quat).as_matrix()

    se3 = np.eye(4)
    se3[:3, :3] = rotmat
    se3[:3, 3] = translation
    return se3


class ControlMode(IntEnum):
    """Control mode enumeration."""

    NONE = 0x00
    CURRENT = 0x01
    EFFORT = 0x02
    VELOCITY = 0x03
    POSITION = 0x04
    PD = 0x09
    OTHER = 0x0A


class ControlGroup(tuple, Enum):
    """Control group enumeration. Each group is a tuple of (start, num_joints). Available groups are: ALL, LEFT_LEG, RIGHT_LEG, WAIST, HEAD, LEFT_ARM, RIGHT_ARM, LOWER, UPPER, UPPER_EXTENDED."""

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

    @classmethod
    def from_string(cls, s: str):
        return getattr(cls, s.upper())


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
    start: np.ndarray
    end: np.ndarray
    duration: float

    def at(self, t: float):
        return self.start + (self.end - self.start) * t / self.duration

    def finished(self, t: float):
        return t >= self.duration
