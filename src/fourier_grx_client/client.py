from __future__ import annotations

import json
import os
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
from .utils import ControlGroup, ControlMode, Serde, Trajectory, se3_to_xyzquat
from .zenoh_utils import ZenohSession

m.patch()

zenoh.init_logger()

np.set_printoptions(precision=2, suppress=True)

logger = logger.opt(ansi=True)


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
            namespace (str, optional): Robot namespace. If not provided, it will try to use the `robot_id` provided in the environment variable `GRX_ROBOT_ID`. If the environment variable is not set, it will try to load the robot_id from the file `~/.fourier/robot_id.yaml`. If the file does not exist, it will fall back to the default namespace `gr`. Defaults to None.

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
            namespace = self._retrieve_namespace()

        logger.info("RobotClient starting...")

        zenoh_config = zenoh.Config()
        zenoh_config.insert_json5(zenoh.config.CONNECT_KEY, json.dumps([f"tcp/{server_ip}:7447"]))
        zenoh_config.insert_json5("scouting/multicast/enabled", "false")
        zenoh_config.insert_json5("scouting/gossip/enabled", "false")

        super().__init__(prefix=namespace, config=zenoh_config)

        self.freq = freq
        self.ctrl_dt = 1 / 200  # TODO: config and doc

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
        self._states: dict[str, defaultdict[str, list | None]] = {
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
            "impedance": self.session.declare_publisher(
                keyexpr=f"{self.prefix}/control/impedance",
                priority=zenoh.Priority.REAL_TIME(),
                congestion_control=zenoh.CongestionControl.DROP(),
            ),
            # TODO: for continuous ik target
            # "ik_target": self.session.declare_publisher(
            #     keyexpr=f"{self.prefix}/control/ik_target",
            #     priority=zenoh.Priority.REAL_TIME(),
            #     congestion_control=zenoh.CongestionControl.DROP(),
            # ),
        }

        # sleep for a while to wait for the subscriber to receive the initial states
        logger.info("Waiting for initial states...")
        time.sleep(0.5)
        while self.joint_positions is None:
            logger.info("Waiting for joint positions...")
            time.sleep(0.1)

        logger.success(f"RobotClient started with namespace: {namespace}")

    @staticmethod
    def _retrieve_namespace() -> str:
        """Get the robot namespace. If the environment variable `GRX_ROBOT_ID` is set, it will use the value as the robot_id. Otherwise, it will try to load the robot_id from the file `~/.fourier/robot_id.yaml`. If the file does not exist, it will fall back to the default namespace `gr`.

        Returns:
            str: The robot namespace.
        """
        robot_id = os.getenv("GRX_ROBOT_ID")
        if robot_id is not None:
            logger.info(
                f"Loaded <blue>robot_id</blue>  from environment variable GRX_ROBOT_ID: {robot_id}. Using namespace: gr/{robot_id}"
            )
            namespace = f"gr/{robot_id}"
            return namespace

        id_path = Path.home() / ".fourier" / "robot_id.yaml"
        logger.warning(f"No namespace provided, trying to load the <blue>robot_id</blue>  from {id_path}.")

        if not id_path.exists():
            namespace = "gr"
            logger.warning(
                "No <blue>robot_id</blue> found, will use the default namespace: gr, which is <red>not recommended</red>."
            )

            return namespace

        robot_id = OmegaConf.load(id_path)["robot_id"]  # type: ignore
        namespace = f"gr/{robot_id}"
        logger.info(
            f"Loaded <blue>robot_id</blue> from ~/.fourier/robot_id.yaml: {robot_id}. Using namespace: gr/{robot_id}"
        )

        return namespace

    def _state_callback(self, sample):
        state_value_type = str(sample.key_expr).split("/")[-2]
        state_value_name = str(sample.key_expr).split("/")[-1]
        state_value: list = Serde.unpack(sample.payload)

        if state_value_type not in self._states:
            raise ValueError(f"Unknown state type: {state_value_type}")

        self._states[state_value_type][state_value_name] = state_value

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

    def get_group_velocity(self, group: ControlGroup):
        """Get the joint velocities of a group."""
        return self.joint_velocities[group.slice].copy()

    def get_group_current(self, group: ControlGroup):
        """Get the joint currents of a group."""
        return self.joint_currents[group.slice].copy()

    def get_group_effort(self, group: ControlGroup):
        """Get the joint efforts of a group."""
        return self.joint_efforts[group.slice].copy()

    @property
    def joint_positions(self):
        """Get the current joint positions of the robot. The joint positions are in radians."""
        position = np.deg2rad(self._states["joint"]["position"])
        return position

    @property
    def joint_velocities(self):
        """Get the current joint velocities of the robot. The joint velocities are in radians per second."""
        velocity = np.deg2rad(self._states["joint"]["velocity"])
        return velocity

    @property
    def joint_efforts(self):
        """Get the current joint efforts of the robot."""
        effort = np.asarray(self._states["joint"]["effort"])
        return effort

    @property
    def joint_currents(self):
        """Get the current joint currents of the robot."""
        current = np.asarray(self._states["joint"]["current"])
        return current

    @property
    def imu_quaternion(self):
        """Get the current IMU orientation as a quaternion."""
        quat = np.asarray(self._states["imu"]["quat"])
        return quat

    @property
    def imu_angles(self):
        """Get the current IMU orientation as Euler angles."""
        euler_angle = np.asarray(self._states["imu"]["euler_angle"])
        return euler_angle

    @property
    def imu_angular_velocity(self):
        """Get the current IMU angular velocity."""
        angular_velocity = np.asarray(self._states["imu"]["angular_velocity"])
        return angular_velocity

    @property
    def imu_linear_acceleration(self):
        """Get the current IMU linear acceleration."""
        acceleration = np.asarray(self._states["imu"]["acceleration"])
        return acceleration

    @property
    def base_pose(self):
        """Get the current base pose."""
        estimate_xyz = np.asarray(self._states["base"]["estimate_xyz"])
        return estimate_xyz

    @property
    def base_velocity(self):
        """Get the current base velocity."""
        estimate_xyz_vel = np.asarray(self._states["base"]["estimate_xyz_vel"])
        return estimate_xyz_vel

    @property
    def number_of_joint(self):
        return len(self.joint_positions)

    @property
    def is_moving(self):
        """Whether the robot is currently moving."""
        return self._move_lock.locked()

    @is_moving.setter
    def is_moving(self, value: bool):
        if value and not self._move_lock.locked():
            self._move_lock.acquire()
        elif not value and self._move_lock.locked():
            self._move_lock.release()

    def enable(self):
        """Enable the motors."""
        self.set_enable(True)

    def disable(self):
        """Disable the motors."""
        self.set_enable(False)

    def set_enable(self, enable: bool):
        """Enable or disable the motors."""

        cmd = "ON" if enable else "OFF"
        self._call_service_wait("control/enable", value=str(cmd), timeout=1.0)

    def calibrate_sensors(self):
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
        degrees: bool = False,
    ):
        """Control the joints in a group with the specified control type.

        Args:
            control_type (Literal["position", "velocity", "current"]): The control type to set.
            commands (np.ndarray | list): The control commands to set.
            degrees (bool, optional): Whether the joint positions and velocities are in degrees. Defaults to False.
        """

        # TODO: figure out how to fill in missing velocity and current values
        if control_type == "position":
            commands = np.deg2rad(commands) if degrees else np.asarray(commands)
            self._publish("position", commands)
        elif control_type == "velocity":
            commands = np.deg2rad(commands) if degrees else np.asarray(commands)
            self._publish("velocity", commands)
        elif control_type == "effort":
            self._publish("effort", commands)
        elif control_type == "current":
            self._publish("current", commands)
        return

    def forward_kinematics(self, chain_names: list[str], q: np.ndarray | None = None) -> list[np.ndarray]:
        """Get the end effector pose of the specified chains in base_link frame.

        !!! info
            The available chain names are: `head`, `left_arm`, `right_arm`, with corresponding end effector frames: `head_yaw_link`, `left_end_effector_link`, `right_end_effector_link`, and the transformation matrices are in the `base_link` frame.

        Args:
            chain_names (list[str]): The chains to get the end effector pose of. Available chain names: 'head', 'left_arm', 'right_arm'.
            q (np.ndarray, optional): The robot confiuration to do forward kinematics in. Defaults to None.

        Returns:
            list: The end effector pose is a list of 4x4 transformation matrices. The order of the matrices is the same as the order of the chain names. The transformation matrix is in the `base_link` frame.
        """

        res = self._call_service_wait(
            "forward_kinematics",
            value=Serde.pack({"chain_names": chain_names, "q": q}),
            timeout=0.1,
        )

        logger.debug(f"FK result for {chain_names}: {res}")
        return res  # type: ignore

    def inverse_kinematics(
        self,
        chain_names: list[str],
        targets: list[np.ndarray],
        move=False,
        dt: float = 0.005,
        velocity_scaling_factor: float = 1.0,
        convergence_threshold: float = 1e-8,
    ):
        """Get the joint positions for the specified chains to reach the target pose in base_link frame.

        Args:
            chain_names (list[str]): The chains to get the joint positions for. Available chain names: 'head', 'left_arm', 'right_arm'.
            targets (list[np.ndarray]): The target poses in base_link frame.
            move (bool, optional): Whether to move the robot to the target pose. Defaults to False.
            dt (float): The time step for the inverse kinematics.

        Returns:
            np.ndarray: The joint positions to reach the target pose (in radians).
        """

        if move:
            self.is_moving = True

        res = self._call_service_wait(
            "inverse_kinematics",
            value=Serde.pack(
                {
                    "move": move,
                    "chain_names": chain_names,
                    "targets": targets,
                    "velocity_scaling_factor": velocity_scaling_factor,
                    "convergence_threshold": convergence_threshold,
                    "dt": dt,
                }
            ),
            timeout=dt / velocity_scaling_factor * 100,
        )

        logger.debug("IK Finished")
        if move:
            self.is_moving = False

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

    def liveness_check(self, timeout: float = 1.0):
        """Check if the robot server is alive.

        Args:
            timeout (float, optional): Timeout in seconds. Defaults to 1.0.

        Raises:
            FourierConnectionError: If the connection to the server failed.
        """

        info = self._call_service_wait("robot/info", timeout=timeout)
        if info is None:
            raise FourierConnectionError(
                "Failed to connect to the robot server.  Please make sure the server is running."
            )
        logger.info(f"Connected to robot server: {info}")

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
        self._publish("position", self.joint_positions)

    def move_joints(
        self,
        group: ControlGroup | list | str,
        positions: np.ndarray | list,
        duration: float = 0.0,
        degrees: bool = False,
        blocking: bool = True,
        gravity_compensation: bool = False,
    ):
        """Move in joint space with time duration.

        Move in joint space with time duration in a separate thread. Can be aborted using `abort()`. Can be blocking.
        If the duration is set to 0, the joints will move in their maximum speed without interpolation.

        ???+ info "GR1T2 Joint Order"
            The joints order is as follows:
                0: left_hip_roll_joint
                  1: left_hip_yaw_joint
                  2: left_hip_pitch_joint
                  3: left_knee_pitch_joint
                  4: left_ankle_pitch_joint
                  5: left_ankle_roll_joint
              6: right_hip_roll_joint
                  7: right_hip_yaw_joint
                  8: right_hip_pitch_joint
                  9: right_knee_pitch_joint
                  10: right_ankle_pitch_joint
                  11: right_ankle_roll_joint
              12: waist_yaw_joint
                  13: waist_pitch_joint
                  14: waist_roll_joint
              15: head_pitch_joint
                  16: head_roll_joint
                  17: head_yaw_joint
              18: left_shoulder_pitch_joint
                  19: left_shoulder_roll_joint
                  20: left_shoulder_yaw_joint
                  21: left_elbow_pitch_joint
                  22: left_wrist_yaw_joint
                  23: left_wrist_roll_joint
                  24: left_wrist_pitch_joint
              25: right_shoulder_pitch_joint
                  26: right_shoulder_roll_joint
                  27: right_shoulder_yaw_joint
                  28: right_elbow_pitch_joint
                  29: right_wrist_yaw_joint
                  30: right_wrist_roll_joint
                  31: right_wrist_pitch_joint


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
            degrees (bool, optional): Whether the joint positions are in degrees. Defaults to False.
            blocking (bool, optional): If True, block until the move is completed. Defaults to True.
            gravity_compensation (bool, optional): Whether to enable gravity compensation. Defaults to False.
        """
        positions = np.deg2rad(positions) if degrees else np.asarray(positions)
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

        if gravity_compensation:
            with self._move_lock:
                self._publish("impedance", Serde.pack({"position": dest_pos}))
            return

        if duration == 0:
            self._publish("position", dest_pos)
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
                    self._publish("position", pos)
                    time.sleep(1 / self.freq)

            self._abort_event.clear()

        if not blocking:
            thread = threading.Thread(name="RobotClient.move_joints", target=task)
            thread.daemon = True
            thread.start()
        else:
            task()

    def movel(
        self,
        sides: list[Literal["left", "right"]],
        target_poses: list[np.ndarray | list],
        max_velocity: float | None = None,
        max_acceleration: float | None = None,
        max_jerk: float | None = None,
        move: bool = False,
    ):
        """Move the specified arm to the target position.

        Args:
            sides (list[Literal["left", "right"]]): Sides of the arms to move. Can be "left" or "right".
            target_poses (list[np.ndarray | list]): Desired end effector poses.
            max_velocity (float | None, optional): Max velocity during trajectory generation. Defaults to None.
            max_acceleration (float | None, optional): Max acceleration during trajectory generation. Defaults to None.
            max_jerk (float | None, optional): Max jerk during trajectory generation. Defaults to None.
            move (bool, optional): Whether to execute the trajectory. Defaults to False.

        Return:
            list: The trajectory to reach the target pose.
        """

        curr_poses = self.forward_kinematics([f"{side}_arm" for side in sides])
        curr_poses = [se3_to_xyzquat(pose.copy()) for pose in curr_poses]

        traj = []
        start = time.time()

        for i, target_pose in enumerate(target_poses):
            if target_pose.shape == (4, 4):
                target_poses[i] = se3_to_xyzquat(target_pose)

        traj: list[np.ndarray] = self._call_service_wait(
            "control/movel",
            value=Serde.pack(
                {
                    "sides": sides,
                    "start_poses": curr_poses,
                    "end_poses": target_poses,
                    "max_velocity": max_velocity,
                    "max_acceleration": max_acceleration,
                    "max_jerk": max_jerk,
                    "dt": self.ctrl_dt,
                }
            ),
            timeout=1000,
        )  # type: ignore

        print(f"Time: {time.time() - start}")

        if traj is None:
            logger.warning("Failed to generate trajectory.")
            return

        if move:
            with self._move_lock:
                for pos in track(
                    traj,
                    description="Moving...",
                    total=len(traj),
                ):
                    if self._abort_event.is_set():
                        self._abort_event.clear()
                        break
                    dest_pos = self.joint_positions.copy()
                    dest_pos[ControlGroup.WAIST.slice] = pos[ControlGroup.WAIST.slice]
                    dest_pos[-14:] = pos[-14:]

                    self._publish("impedance", Serde.pack({"position": dest_pos}))
                    time.sleep(self.ctrl_dt)

        return traj

    def movej(
        self,
        side: Literal["left", "right"],
        target_position: np.ndarray | list,
        target_velocity: np.ndarray | list | None = None,
        target_acceleration: np.ndarray | list | None = None,
        max_velocity: np.ndarray | list | None = None,
        max_acceleration: np.ndarray | list | None = None,
        max_jerk: np.ndarray | list | None = None,
        degrees: bool = False,
        move: bool = False,
    ):
        """move the specified arm to the target position.

        Args:
            side (Literal["left", "right"]): Side of the arm to move. Can be "left" or "right".
            target_position (np.ndarray | list): Desired joint position.
            target_velocity (np.ndarray | list | None, optional): Desired joint velocity. Defaults to None. If None, the robot will estimate the velocity based on the target position.
            target_acceleration (np.ndarray | list | None, optional): Desired acceleration. Defaults to None.
            max_velocity (np.ndarray | list | None, optional): Max velocity during trajectory generation. Defaults to None. If None, the robot will use the default max velocity at 5 rad/s.
            max_acceleration (np.ndarray | list | None, optional): Max acceleration during trajectory generation. Defaults to None. If None, the robot will use the default max acceleration at 10 rad/s^2.
            max_jerk (np.ndarray | list | None, optional): Max jerk during trajectory generation. Defaults to None. If None, the robot will use the default max jerk at 50 rad/s^3.
            degrees (bool, optional): True if the input is in degrees. Defaults to False.
            move (bool, optional): Whether to execute the trajectory. Defaults to False.

        Returns:
            np.ndarray: The joint positions to reach the target pose (in radians).
        """
        if degrees:
            position = np.deg2rad(target_position)
            velocity = np.deg2rad(target_velocity) if target_velocity is not None else None
            acceleration = np.deg2rad(target_acceleration) if target_velocity is not None else None
            max_velocity = np.deg2rad(max_velocity) if max_velocity is not None else None
            max_acceleration = np.deg2rad(max_acceleration) if max_acceleration is not None else None
            max_jerk = np.deg2rad(max_jerk) if max_jerk is not None else None
        else:
            position = np.asarray(target_position)
            velocity = np.asarray(target_velocity) if target_velocity is not None else None
            acceleration = np.asarray(target_acceleration) if target_acceleration is not None else None
            max_velocity = np.asarray(max_velocity) if max_velocity is not None else None
            max_acceleration = np.asarray(max_acceleration) if max_acceleration is not None else None
            max_jerk = np.asarray(max_jerk) if max_jerk is not None else None

        traj = self._call_service_wait(
            "control/movej",
            value=Serde.pack(
                {
                    "side": side,
                    "position": position,
                    "velocity": velocity,
                    "acceleration": acceleration,
                    "max_velocity": max_velocity,
                    "max_acceleration": max_acceleration,
                    "max_jerk": max_jerk,
                }
            ),
        )

        if traj is None:
            logger.warning("Failed to generate trajectory.")
            return

        if move:
            with self._move_lock:
                for pos, _ in track(
                    traj,
                    description="Moving...",
                    total=len(traj),
                ):
                    if self._abort_event.is_set():
                        self._abort_event.clear()
                        break
                    dest_pos = self.joint_positions.copy()

                    if side == "left":
                        dest_pos[ControlGroup.LEFT_ARM.slice] = pos
                    else:
                        dest_pos[ControlGroup.RIGHT_ARM.slice] = pos
                    self._publish("impedance", Serde.pack({"position": dest_pos}))
                    time.sleep(self.ctrl_dt)

        return traj

    def abort(self):
        self._abort_event.set()
        self._update_pos()  # TODO:  test this

    def play_traj(self, group: ControlGroup, traj: list[np.ndarray], timestamps: list[float] | None = None, dt=0.005):
        """Play a trajectory in joint space."""

        # safely move to the start position
        logger.info("Moving to start position...")
        self.move_joints(group, traj[0], 2.0, blocking=True)
        logger.info("Start position reached.")

        if timestamps is None:
            with self._move_lock:
                for pos in track(
                    traj[1:],
                    description="Moving...",
                    total=len(traj) - 1,
                ):
                    dest_pos = self.joint_positions.copy()
                    if self._abort_event.is_set():
                        self._abort_event.clear()
                        break

                    dest_pos[group.slice] = pos
                    self._publish("position", dest_pos)
                    time.sleep(dt)
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
