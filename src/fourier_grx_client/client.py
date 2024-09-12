from __future__ import annotations

import json
import sys
import threading
import time
from collections import defaultdict
from pathlib import Path
from typing import Literal

import msgpack_numpy as m
import numpy as np
import zenoh
from loguru import logger
from omegaconf import OmegaConf
from rich.progress import track

from .constants import DEFAULT_POSITIONS
from .exceptions import FourierConnectionError, FourierValueError
from .utils import ControlGroup, ControlMode, Serde, Trajectory
from .zenoh_utils import ZenohSession

m.patch()

zenoh.init_logger()

np.set_printoptions(precision=2, suppress=True)


class RobotClient(ZenohSession):
    """Client class for GR series robots.
    Example:

        >>> from fourier_grx_client import RobotClient
        >>> r = RobotClient(namespace="gr/my_awesome_robot", server_ip="192.168.6.6")
    """

    default_positions = DEFAULT_POSITIONS
    default_group_positions = {group: DEFAULT_POSITIONS[group.slice].copy() for group in ControlGroup}

    def __init__(
        self,
        freq: int = 400,
        namespace: str | None = None,
        server_ip: str = "localhost",
        verbose: bool = False,
    ):
        """The client class for GR series robots.

        Args:
            server_ip (str, optional): IP address of the grx server. Please make sure to properly setup the firewall to allow port 7447. Defaults to "localhost".
            freq (int, optional): Robot state update frequency. Usually the user doesn't need to modify this. Defaults to 400.
            namespace (str, optional): Robot namespace. If not provided, it will try to use the `robot_id` provided in `~/.fourier/robot_id.yaml`. If the file does not exist, it will be created using the computer username. Defaults to None.

        Raises:
            FourierConnectionError: If the connection to the server failed.
        """
        if verbose:
            logger.remove()
            logger.add(sys.stderr, level="DEBUG")
        else:
            logger.remove()
            logger.add(sys.stderr, level="INFO")
        if namespace is None:
            robot_serial_number_store_path = Path.home() / ".fourier" / "robot_id.yaml"
            if not robot_serial_number_store_path.exists():
                # create the folder
                robot_serial_number_store_path.parent.mkdir(parents=True, exist_ok=True)

                # write the default serial number
                with open(robot_serial_number_store_path, "w") as f:
                    f.write(f"robot_id: robot_{Path.home().name}")

            robot_id = OmegaConf.load(robot_serial_number_store_path)["robot_id"]
            namespace = f"gr/{robot_id}"

        logger.success(f"RobotClient namespace = {namespace}")

        # -------------------------

        zenoh_config = zenoh.Config()
        zenoh_config.insert_json5(zenoh.config.CONNECT_KEY, json.dumps([f"tcp/{server_ip}:7447"]))
        zenoh_config.insert_json5("scouting/multicast/enabled", "false")
        zenoh_config.insert_json5("scouting/gossip/enabled", "false")

        super().__init__(prefix=namespace, config=zenoh_config)

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
        self.states: dict[str, defaultdict[str, list | None]] = {
            "imu": defaultdict(lambda: None),
            "joint": defaultdict(lambda: None),
            "base": defaultdict(lambda: None),
        }

        # subscribers for the robot states
        self._subscribers = {
            "imu": self.session.declare_subscriber(
                keyexpr=f"{self.prefix}/imu/*",
                handler=self._state_callback,
                reliability=zenoh.Reliability.BEST_EFFORT(),
            ),
            "joint": self.session.declare_subscriber(
                keyexpr=f"{self.prefix}/joint/*",
                handler=self._state_callback,
                reliability=zenoh.Reliability.BEST_EFFORT(),
            ),
            "base": self.session.declare_subscriber(
                keyexpr=f"{self.prefix}/base/*",
                handler=self._state_callback,
                reliability=zenoh.Reliability.BEST_EFFORT(),
            ),
        }

        # publishers for the robot control
        self._publishers = {
            "position": self.session.declare_publisher(
                keyexpr=f"{self.prefix}/control/joints/position",
                priority=zenoh.Priority.REAL_TIME(),
                congestion_control=zenoh.CongestionControl.DROP(),
            ),
            "velocity": self.session.declare_publisher(
                keyexpr=f"{self.prefix}/control/joints/velocity",
                priority=zenoh.Priority.REAL_TIME(),
                congestion_control=zenoh.CongestionControl.DROP(),
            ),
            "effort": self.session.declare_publisher(
                keyexpr=f"{self.prefix}/control/joints/effort",
                priority=zenoh.Priority.REAL_TIME(),
                congestion_control=zenoh.CongestionControl.DROP(),
            ),
            "current": self.session.declare_publisher(
                keyexpr=f"{self.prefix}/control/joints/current",
                priority=zenoh.Priority.REAL_TIME(),
                congestion_control=zenoh.CongestionControl.DROP(),
            ),
        }

        # sleep for a while to wait for the subscriber to receive the initial states
        time.sleep(0.5)
        while self.states["joint"]["position"] is None:
            logger.info("Waiting for joint positions...")
            time.sleep(0.1)

        logger.success("RobotClient OK!")

    def _state_callback(self, sample):
        state_value_type = str(sample.key_expr).split("/")[-2]
        state_value_name = str(sample.key_expr).split("/")[-1]
        state_value: list = Serde.unpack(sample.payload)

        if state_value_type not in self.states:
            raise ValueError(f"Unknown state type: {state_value_type}")

        self.states[state_value_type][state_value_name] = state_value

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
    def joint_velocity(self):
        """Get the current joint velocities of the robot."""
        velocity = np.asarray(self.states["joint"]["velocity"])
        return velocity

    @property
    def joint_effort(self):
        """Get the current joint efforts of the robot."""
        effort = np.asarray(self.states["joint"]["effort"])
        return effort

    @property
    def joint_current(self):
        """Get the current joint currents of the robot."""
        current = np.asarray(self.states["joint"]["current"])
        return current

    @property
    def number_of_joint(self):
        return len(self.joint_positions)

    @property
    def is_moving(self):
        """Whether the robot is currently moving."""
        return self._move_lock.locked()

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

    def get_gains(self):
        """Get the control gains for all joints."""

        res = self._call_service_wait(
            "control/gains",
            timeout=0.1,
        )
        return res

    def get_control_modes(self):
        """Get the control modes for all joints."""

        res = self._call_service_wait(
            "control/modes",
            timeout=0.1,
        )
        return res

    def set_control_modes(self, control_mode: ControlMode | list[ControlMode] | None = None):
        """Set the control modes for all joints.

        Args:
            control_mode (ControlMode | list[ControlMode] | None, optional): ControlMode can be PD, VELOCITY, CURRENT. Defaults to None.
        """
        out_control_mode = [ControlMode.PD.value] * self.number_of_joint
        if control_mode is None:
            out_control_mode = [ControlMode.NONE.value] * self.number_of_joint

        if isinstance(control_mode, ControlMode):
            out_control_mode = [control_mode.value] * self.number_of_joint

        if isinstance(control_mode, list) and len(control_mode) == self.number_of_joint:
            out_control_mode = [mode.value for mode in control_mode]

        res = self._call_service_wait(
            "control/modes",
            value=Serde.pack(out_control_mode),
            timeout=0.1,
        )
        return res

    def set_gains(
        self,
        position_control_kp: list[float] | None = None,
        velocity_control_kp: list[float] | None = None,
        velocity_control_ki: list[float] | None = None,
        pd_control_kp: list[float] | None = None,
        pd_control_kd: list[float] | None = None,
    ):
        gains = {}
        if position_control_kp is not None and len(position_control_kp) == self.number_of_joint:
            gains["position_control_kp"] = position_control_kp
        if velocity_control_kp is not None and len(velocity_control_kp) == self.number_of_joint:
            gains["velocity_control_kp"] = velocity_control_kp
        if velocity_control_ki is not None and len(velocity_control_ki) == self.number_of_joint:
            gains["velocity_control_ki"] = velocity_control_ki
        if pd_control_kp is not None and len(pd_control_kp) == self.number_of_joint:
            gains["pd_control_kp"] = pd_control_kp
        if pd_control_kd is not None and len(pd_control_kd) == self.number_of_joint:
            gains["pd_control_kd"] = pd_control_kd

        res = self._call_service_wait(
            "control/gains",
            value=Serde.pack(gains),
            timeout=0.1,
        )
        return res

    def control_joints(
        self,
        control_type: Literal["position", "velocity", "effort", "current"],
        commands: np.ndarray | list,
    ):
        """Control the joints in a group with the specified control type.

        Args:
            control_type (Literal["position", "velocity", "current"]): The control type to set.
            commands (np.ndarray | list): The control commands to set.
        """

        # TODO: figure out how to fill in missing velocity and current values
        if control_type == "position":
            self._publish("position", commands)
        elif control_type == "velocity":
            self._publish("velocity", commands)
        elif control_type == "effort":
            self._publish("effort", commands)
        elif control_type == "current":
            self._publish("current", commands)
        return

    def forward_kinematics(self, chain_names: list[str], q: np.ndarray | None = None):
        """Get the end effector pose of the specified chains.

        Args:
            chain_names (list[str]): The chains to get the end effector pose of. Available chain names: 'head', 'left_arm', 'right_arm'.
            q (np.ndarray, optional): The robot confiuration to do forward kinematics in. Defaults to None.

        Returns:
            list: The end effector pose of the specified chains.
        """

        res = self._call_service_wait(
            "forward_kinematics",
            value=Serde.pack({"chain_names": chain_names, "q": q}),
            timeout=0.1,
        )
        return res

    def inverse_kinematics(self, chain_names: list[str], targets: list[np.ndarray], dt: float, degrees=True):
        """Get the joint positions for the specified chains to reach the target pose.

        Args:
            chain_names (list[str]): The chains to get the joint positions for. Available chain names: 'head', 'left_arm', 'right_arm'.
            targets (list[np.ndarray]): The target poses.
            dt (float): The time step for the inverse kinematics.

        Returns:
            np.ndarray: The joint positions to reach the target pose.
        """

        res = self._call_service_wait(
            "inverse_kinematics",
            value=Serde.pack({"chain_names": chain_names, "targets": targets, "dt": dt}),
            timeout=0.1,
        )

        if res is not None and degrees:
            res = np.rad2deg(res)
        return res

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

    def liveness_check(self):
        """Check if the robot server is alive."""

        info = self._call_service_wait("robot/info", timeout=1.0)
        if info is None:
            raise FourierConnectionError(
                "Failed to connect to the robot server.  Please make sure the server is running."
            )

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
        blocking: bool = True,
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
            blocking (bool, optional): If True, block until the move is completed. Defaults to True.
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
                    self._publish("position", pos)
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
                self._publish("position", pos)
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
