from __future__ import annotations

import json
import sys
import threading
import time
from collections import defaultdict
from typing import Optional

import msgpack_numpy as m
import numpy as np
import zenoh
from loguru import logger
from rich.console import Console
from rich.progress import track

from .constants import DEFAULT_POSITIONS
from .exceptions import FourierConnectionError, FourierValueError
from .utils import ControlGroup, Serde, Trajectory
from .zenoh import ZenohSession

m.patch()

zenoh.init_logger()
console = Console()
log = console.log
print = console.print

np.set_printoptions(precision=2, suppress=True)


class RobotClient(ZenohSession):
    """Client class for GR series robots.
    Example:

        >>> from fourier_grx_client import *
        >>> r = RobotClient(namespace="gr/my_awesome_robot", server_ip="192.168.6.6")
    """

    default_positions = DEFAULT_POSITIONS
    default_group_positions = {group: DEFAULT_POSITIONS[group.slice].copy() for group in ControlGroup}

    def __init__(self, freq: int = 400, namespace: str = "gr", server_ip: str = "localhost", verbose: bool = False):
        """The client class for GR series robots.

        Args:
            server_ip (str, optional): IP address of the grx server. Please make sure to properly setup the firewall to allow port 7447. Defaults to "localhost".
            freq (int, optional): Robot state update frequency. Usually the user doesn't need to modify this. Defaults to 400.
            namespace (str, optional): Robot namespace. Defaults to "gr".
            verbose (bool, optional): Whether to print debug messages. Defaults to False.

        Raises:
            FourierConnectionError: If the connection to the server failed.
        """
        if verbose:
            logger.remove()
            logger.add(sys.stderr, level="DEBUG")
        else:
            logger.remove()
            logger.add(sys.stderr, level="INFO")
        conf = zenoh.Config()
        conf.insert_json5(zenoh.config.CONNECT_KEY, json.dumps([f"tcp/{server_ip}:7447"]))
        conf.insert_json5("scouting/multicast/enabled", "false")
        conf.insert_json5("scouting/gossip/enabled", "false")
        super().__init__(namespace, conf)
        self.freq = freq

        self._abort_event = threading.Event()
        self._move_lock = threading.Lock()
        self.server_connected: bool = False

        try:
            self.liveness_check()
        except FourierConnectionError as ex:
            logger.error("Failed to connect to the robot server. Exiting...")
            self.session.close()
            raise ex

        self.server_connected = True

        # dict to store the states of the robot, updated by the subscriber callback
        self.states: dict[str, defaultdict[str, Optional[list]]] = {
            "imu": defaultdict(lambda: None),
            "joint": defaultdict(lambda: None),
            "base": defaultdict(lambda: None),
        }

        # subscribers for the robot states
        self._subscribers = {
            "imu": self.session.declare_subscriber(
                f"{self.prefix}/imu/*",
                self._sensor_callback,
                reliability=zenoh.Reliability.BEST_EFFORT(),
            ),
            "joint": self.session.declare_subscriber(
                f"{self.prefix}/joint/*",
                self._sensor_callback,
                reliability=zenoh.Reliability.BEST_EFFORT(),
            ),
            "base": self.session.declare_subscriber(
                f"{self.prefix}/base/*",
                self._sensor_callback,
                reliability=zenoh.Reliability.BEST_EFFORT(),
            ),
        }

        # publishers for the robot control
        self._publishers = {
            "positions": self.session.declare_publisher(
                f"{self.prefix}/control/joints/position",
                priority=zenoh.Priority.REAL_TIME(),
                congestion_control=zenoh.CongestionControl.DROP(),
            ),
        }

        # sleep for a while to wait for the subscriber to receive the initial states
        time.sleep(0.5)

    def get_group_position(self, group: ControlGroup):
        """Get the joint positions of a group."""

        return self.joint_positions[group.slice].copy()

    def get_group_position_by_name(self, name: str):
        """Get the joint positions of a group by its name.

        Args:
            name (str): The name of the group. Available group names: 'left_leg', 'right_leg', 'waist', 'head', 'left_arm', 'right_arm'.
        """

        try:
            group = ControlGroup[name.upper()]
            return self.get_group_position(group)
        except KeyError as ex:
            raise FourierValueError(f"Unknown group name: {name}") from ex

    @property
    def joint_positions(self):
        """Get the current joint positions of the robot."""
        position = np.asarray(self.states["joint"]["position"])
        return position

    @property
    def joint_velocities(self):
        """Get the current joint velocities of the robot."""
        velocity = np.asarray(self.states["joint"]["velocity"])
        return velocity

    @property
    def is_moving(self):
        """Whether the robot is currently moving."""
        return self._move_lock.locked()

    def liveness_check(self):
        """Check if the robot server is alive."""

        info = self._call_service_wait("robot/info", timeout=1.0)
        if info is None:
            raise FourierConnectionError(
                "Failed to connect to the robot server.  Please make sure the server is running."
            )

    def _sensor_callback(self, sample):
        sensor_type = str(sample.key_expr).split("/")[-2]
        sensor_name = str(sample.key_expr).split("/")[-1]
        sensor_reading: list = Serde.unpack(sample.payload)
        if sensor_type not in self.states:
            raise ValueError(f"Unknown sensor type: {sensor_type}")
        self.states[sensor_type][sensor_name] = sensor_reading

        # print(f"Received '{sample.key_expr}': '{joint_value}, {joint_value.shape}'")

    def set_enable(self, enable: bool):
        """Enable or disable the motors."""

        cmd = "ON" if enable else "OFF"
        self._call_service_wait("control/enable", value=str(cmd), timeout=1.0)

    def enable(self):
        """Enable the motors."""
        self.set_enable(True)

    def disable(self):
        """Disable the motors."""
        self.set_enable(False)

    def set_home(self):
        """Get sensor offsets and save to `sensor_offset.json`"""

        self._call_service_wait("control/set_home", timeout=11.0)

    def get_transform(self, target_frame: str, source_frame: str, q: np.ndarray | None = None):
        """Get the transformation matrix between two frames in configuration `q`. If `q` is None, the current joint positions are used.

        Args:
            target_frame (str): Name of the frame to get the pose of.
            source_frame (str): Name of the frame to get the pose in.
            q (np.ndarray, optional): The robot confiuration to do forward kinematics in. Defaults to None.

        Returns:
            np.ndarray: The transformation matrix.
        """

        res = self._call_service_wait(
            f"tf/{source_frame}/{target_frame}",
            value=Serde.pack(q) if q is not None else None,
            timeout=1.0,
        )
        return np.array(res)

    def list_frames(self):
        """List all available frames."""

        res = self._call_service_wait(
            "tf/list",
            timeout=1.0,
        )
        return res

    def set_gains(self, kp: np.ndarray, kd: np.ndarray, control_mode: np.ndarray | None = None):
        """Set the gains for the joints.


        Args:
            kp (np.ndarray): The proportional gains.
            kd (np.ndarray): The derivative gains.
            control_mode (np.ndarray, optional): The control mode for each joint. Defaults to None.

        """
        kp = np.asanyarray(kp)
        kd = np.asanyarray(kd)
        if control_mode is None:
            control_mode = np.array([9] * 32)
        else:
            control_mode = np.asanyarray(control_mode, dtype=int)

        """Currently only support position control"""

        if self.joint_positions is None:
            logger.warning("Joint positions not available, retrying...")
            time.sleep(0.1)

        if kp.shape != (self.joint_positions.shape[0],) or kd.shape != (self.joint_positions.shape[0],):
            raise ValueError(
                f"Invalid kp/kd shape: {kp.shape}/{kd.shape}, expected: {(self.joint_positions.shape[0],)}"
            )

        res = self._call_service_wait(
            "control/gains",
            value=Serde.pack(
                {
                    "control_mode": list(control_mode),
                    "kp": list(kp),
                    "kd": list(kd),
                }
            ),
            timeout=0.1,
        )
        return res

    def reboot(self):
        """Reboot the motors."""

        self._call_service_wait("control/reboot", timeout=10.0)
        self._update_pos()
        time.sleep(0.1)
        self._update_pos()
        time.sleep(0.1)
        self._update_pos()

    def _update_pos(self):
        """Update the joint positions command to the measured values.

        This is useful when the robot is moved manually and the joint positions are not updated.
        """
        # silently ignore if the robot is moving
        if self._move_lock.locked():
            return
        self._publish("positions", self.states["joint"]["position"])

    def move_joints(
        self,
        group: ControlGroup | list | str,
        positions: np.ndarray | list,
        duration: float = 0.0,
        degrees: bool = True,
        blocking: bool = False,
    ):
        """Move in joint space with time duration.

        Move in joint space with time duration in a separate thread. Can be aborted using `abort()`. Can be blocking.
        If the duration is set to 0, the joints will move in their maximum speed without interpolation.

        ???+ info "Joint Order"
            The joints order is as follows:
                0: l_hip_roll, 1: l_hip_yaw, 2: l_hip_pitch, 3: l_knee_pitch, 4: l_ankle_pitch, 5: l_ankle_roll, 6: r_hip_roll, 7: r_hip_yaw, 8: r_hip_pitch, 9: r_knee_pitch, 10: r_ankle_pitch, 11: r_ankle_roll, 12: joint_waist_yaw, 13: joint_waist_pitch, 14: joint_waist_roll, 15: joint_head_pitch, 16: joint_head_roll, 17: joint_head_yaw, 18: l_shoulder_pitch, 19: l_shoulder_roll, 20: l_shoulder_yaw, 21: l_elbow_pitch, 22: l_wrist_yaw, 23: l_wrist_roll, 24: l_wrist_pitch, 25: r_shoulder_pitch, 26: r_shoulder_roll, 27: r_shoulder_yaw, 28: r_elbow_pitch, 29: r_wrist_yaw, 30: r_wrist_roll, 31: r_wrist_pitch

        Example:

            >>> # Move the left arm to a specific position
            >>> r.move_joints("left_arm", [0, 0, 0, 20, 0, 0, 0], degrees=True)

            >>> # or use the ControlGroup enum
            >>> r.move_joints(ControlGroup.LEFT_ARM, [0, 0, 0, 20, 0, 0, 0], degrees=True)

            >>> # or use indices, with radians instead of degrees
            >>> r.move_joints([23, 24], [0.17, 0.17], degrees=False)

        Args:
            group (ControlGroup | list | str): The group of joints to move, specified by a string or a ControlGroup enum, or a list of joint indices.
            positions (np.ndarray[float]): target joint position in degrees.
            duration (float, optional): Time duration in seconds. If set to 0, the joints will move in their maximum speed without interpolation. Defaults to 0.0.
            degrees (bool, optional): Whether the joint positions are in degrees. Defaults to True.
            blocking (bool, optional): If True, block until the move is completed. Defaults to False.
        """
        if not degrees:
            positions = np.rad2deg(positions)
        else:
            positions = np.asarray(positions)
        dest_pos = self.joint_positions.copy()

        if isinstance(group, ControlGroup):
            if positions.shape != (group.num_joints,):
                raise ValueError(f"Invalid joint position shape: {positions.shape}, expected: {(group.num_joints,)}")
            dest_pos[group.slice] = positions
        elif isinstance(group, list):
            if len(group) != len(positions):
                raise ValueError(f"Invalid joint position shape: {positions.shape}, expected: {(len(group),)}")
            dest_pos[group] = positions
        elif isinstance(group, str):
            try:
                dest_pos[ControlGroup[group.upper()].slice] = positions
            except KeyError as ex:
                raise FourierValueError(f"Unknown group name: {group}") from ex

        # TODO: automatic interpolation

        if self.is_moving:
            logger.warning("Move already in progress, abort.")
            return

        if duration == 0:
            self._publish("positions", dest_pos)
            return

        def task():
            with self._move_lock:
                trajectory = Trajectory(
                    start=self.joint_positions,
                    end=dest_pos,
                    duration=duration,
                )

                start_time = time.time()
                while not (trajectory.finished(t := time.time() - start_time) or self._abort_event.is_set()):
                    pos = trajectory.at(t)
                    self._publish("positions", pos)
                    time.sleep(1 / self.freq)

            self._abort_event.clear()

        if not blocking:
            thread = threading.Thread(name="RobotClient.move_joints", target=task)
            thread.daemon = True
            thread.start()
        else:
            task()

    def abort(self):
        self._abort_event.set()
        self._update_pos()  # TODO:  test this

    def play_traj(self, traj: list[np.ndarray], timestamps: list[float] | None = None):
        """Play a trajectory in joint space."""

        # safely move to the start position
        logger.info("Moving to start position...")
        self.move_joints(ControlGroup.ALL, traj[0], 2.0, blocking=True)
        logger.info("Start position reached.")

        if timestamps is None:
            with self._move_lock:
                for pos in track(
                    traj[1:],
                    description="Moving...",
                    total=len(traj) - 1,
                ):
                    if self._abort_event.is_set():
                        self._abort_event.clear()
                        break
                    self._publish("positions", pos)
                    time.sleep(1 / self.freq)
            return

        t0 = timestamps[0]
        start_time = time.time()
        with self._move_lock:
            for pos, t in track(
                zip(traj[1:], timestamps[1:], strict=False),
                description="Moving...",
                total=len(traj) - 1,
            ):
                if self._abort_event.is_set():
                    self._abort_event.clear()
                    break
                # self.move_joints(ControlGroup.ALL, pos, t - (time.time() - start_time))
                self._publish("positions", pos)
                if sleep_time := (t - t0) - (time.time() - start_time) > 0:
                    time.sleep(sleep_time)

    def close(self):
        if self.server_connected:
            self.abort()
            # self.set_enable(False)
            time.sleep(0.1)

        super().close()


if __name__ == "__main__":
    import doctest

    doctest.testmod()
