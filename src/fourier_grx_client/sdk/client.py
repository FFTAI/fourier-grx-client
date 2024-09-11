import json
import threading
import time
from collections import defaultdict
from pathlib import Path
from typing import Literal

import numpy
import zenoh
import msgpack_numpy
from omegaconf import OmegaConf
from rich.console import Console
from rich.progress import track

from .logger import Logger
from .constants import DEFAULT_POSITIONS
from .exceptions import FourierConnectionError, FourierValueError
from .utils import FunctionResult, ControlConfig, ControlGroup, ControlMode, Serde, Trajectory
from .zenoh import ZenohSession

msgpack_numpy.patch()

zenoh.init_logger()
console = Console()
log = console.log
print = console.print

numpy.set_printoptions(precision=2, suppress=True)


class RobotClient(ZenohSession):
    """Client class for GR series robots.
    Example:

        >>> from fourier_grx_client import *
        >>> r = RobotClient(namespace="gr/my_awesome_robot", server_ip="192.168.6.6")
    """

    default_positions = DEFAULT_POSITIONS
    default_group_positions = {group: DEFAULT_POSITIONS[group.slice].copy() for group in ControlGroup}

    def __init__(self, freq: int = 400, namespace=None, server_ip: str = "localhost"):
        """The client class for GR series robots.

        Args:
            server_ip (str, optional): IP address of the grx server. Please make sure to properly setup the firewall to allow port 7447. Defaults to "localhost".
            freq (int, optional): Robot state update frequency. Usually the user doesn't need to modify this. Defaults to 400.
            namespace (str, optional): Robot namespace. Defaults to "gr".
            verbose (bool, optional): Whether to print debug messages. Defaults to False.

        Raises:
            FourierConnectionError: If the connection to the server failed.
        """
        # get robot serial number
        # - robot serial number is default stored in "~/.fourier/robot_serial_number.yaml"
        # - if the file does not exist, it will be created using the computer username.
        # - if the file exists, the serial number will be read from the file.
        # - the serial number is used as the robot id.
        # - namespace = "gr" / robot_id
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

        # -------------------------

        zenoh_config = zenoh.Config()
        zenoh_config.insert_json5(zenoh.config.CONNECT_KEY, json.dumps([f"tcp/{server_ip}:7447"]))
        zenoh_config.insert_json5("scouting/multicast/enabled", "false")
        zenoh_config.insert_json5("scouting/gossip/enabled", "false")
        # conf.insert_json5("transport/unicast/lowlatency", "true")
        # conf.insert_json5("transport/unicast/qos/enabled", "false")

        Logger().print_success(f"RobotClient start with namespace: {namespace}")

        super().__init__(prefix=namespace, config=zenoh_config)

        self.freq = freq
        self._abort_event = threading.Event()
        self._move_lock = threading.Lock()
        self.server_connected: bool = False

        if self.liveness_check() == FunctionResult.SUCCESS:
            pass
        else:
            self.session.close()
            raise FourierConnectionError(f"Failed to connect to the robot server at {server_ip}")

        self.server_connected = True

        # dict to store the states of the robot, updated by the subscriber callback
        self.states = {
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

        # # create properties for each joint group
        # for group in ControlGroup:
        #     print(group.name, group.value)
        #     setattr(
        #         RobotClient,
        #         f"{group.name.lower()}_pos",
        #         property(
        #             lambda self: self.joint_positions.copy()[
        #                 group.value[0] : group.value[0] + group.value[1]
        #             ]
        #         ),
        #     )

        # sleep for a while to wait for the subscriber to receive the initial states
        time.sleep(0.5)
        while self.states.get("joint", {}).get("position", None) is None:
            Logger().print_info("Waiting for joint positions...")
            time.sleep(0.1)

        Logger().print_success("RobotClient OK!")

    # ==============================================================================================================

    def _state_callback(self, sample):
        state_value_type = str(sample.key_expr).split("/")[-2]
        state_value_name = str(sample.key_expr).split("/")[-1]
        state_value: list = Serde.unpack(sample.payload)

        if state_value_type not in self.states:
            raise ValueError(f"Unknown state type: {state_value_type}")

        self.states[state_value_type][state_value_name] = state_value

        # print(f"Received '{sample.key_expr}': '{joint_value}, {joint_value.shape}'")

    # ==============================================================================================================

    def get_group_position(self, group: ControlGroup):
        """Get the joint positions of a group."""

        return self.joint_positions[group.slice].copy()

    def get_group_position_by_name(self, name: str):
        """Get the joint positions of a group by its name.

        Args:
            name (str): The name of the group.
                        Available group names: 'left_leg', 'right_leg', 'waist', 'head', 'left_arm', 'right_arm'.
        """

        try:
            group = ControlGroup[name.upper()]
            return self.get_group_position(group)
        except KeyError as ex:
            raise FourierValueError(f"Unknown group name: {name}") from ex

    @property
    def joint_positions(self):
        position = numpy.asarray(self.states["joint"]["position"])
        return position

    @property
    def joint_velocity(self):
        velocity = numpy.asarray(self.states["joint"]["velocity"])
        return velocity

    @property
    def joint_effort(self):
        effort = numpy.asarray(self.states["joint"]["effort"])
        return effort

    @property
    def joint_current(self):
        current = numpy.asarray(self.states["joint"]["current"])
        return current

    @property
    def number_of_joint(self):
        return len(self.joint_positions)

    # @property
    # def joint_control_modes(self):
    #     from fourier_grx.sdk.utils import ControlMode

    #     control_modes = [ControlMode(mode) for mode in self.states["joint"]["control_mode"]]
    #     return control_modes

    # @property
    # def joint_position_control_gains(self):
    #     position_control_kp = numpy.asarray(self.states["joint"]["position_control_kp"])
    #     velocity_control_kp = numpy.asarray(self.states["joint"]["velocity_control_kp"])
    #     velocity_control_ki = numpy.asarray(self.states["joint"]["velocity_control_ki"])
    #     return position_control_kp, velocity_control_kp, velocity_control_ki

    # @property
    # def joint_velocity_control_gains(self):
    #     velocity_control_kp = numpy.asarray(self.states["joint"]["velocity_control_kp"])
    #     velocity_control_ki = numpy.asarray(self.states["joint"]["velocity_control_ki"])
    #     return velocity_control_kp, velocity_control_ki

    # @property
    # def joint_pd_control_gains(self):
    #     pd_control_kp = numpy.asarray(self.states["joint"]["pd_control_kp"])
    #     pd_control_kd = numpy.asarray(self.states["joint"]["pd_control_kd"])
    #     return pd_control_kp, pd_control_kd

    @property
    def is_moving(self):
        """Whether the robot is currently moving."""
        return self._move_lock.locked()

    # def set_gain(self, idx: int, control_config: ControlConfig):
    #     """Set the control gains for a joint.

    #     Args:
    #         idx (int): The joint index.
    #         control_config (ControlConfig): The control configuration.
    #     """

    #     res = self._call_service_wait(
    #         f"control/gain/{idx}",
    #         value=Serde.pack(control_config.to_dict()),
    #         timeout=0.1,
    #     )
    #     return res

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

    def set_control_modes(self, control_mode=None):
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
            position_control_kp=None,
            velocity_control_kp=None,
            velocity_control_ki=None,
            pd_control_kp=None,
            pd_control_kd=None,
    ):
        # if self.joint_positions is None:
        #     Logger().print_warning("Joint positions not available, retrying...")
        #     time.sleep(0.1)

        # if kp.shape != (self.joint_positions.shape[0],) or kd.shape != (self.joint_positions.shape[0],):
        #     raise ValueError(
        #         f"Invalid kp/kd shape: {kp.shape}/{kd.shape}, expected: {(self.joint_positions.shape[0],)}"
        #     )

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
            # group: ControlGroup | list,
            control_type: Literal["position", "velocity", "effort", "current"],
            commands: numpy.ndarray | list,
    ):
        """Control the joints in a group with the specified control type.

        Args:
            # group (ControlGroup | list): The group of joints to control, or a list of joint indices.
            control_type (Literal["position", "velocity", "current"]): The control type to set.
            commands (numpy.ndarray | list): The control commands to set.
        """

        # TODO: figure out how to fill in missing velocity and current values
        # if isinstance(group, ControlGroup):
        #     group = group.slice
        # elif isinstance(group, list):

        if control_type == "position":
            self._publish("position", commands)
        elif control_type == "velocity":
            self._publish("velocity", commands)
        elif control_type == "effort":
            self._publish("effort", commands)
        elif control_type == "current":
            self._publish("current", commands)
        return

    def move_joints(
            self,
            group,
            positions: numpy.ndarray | list,
            duration: float = 0.0,
            degrees: bool = True,
            blocking: bool = False,
    ):
        """Move in joint space with time duration.

        Move in joint space with time duration in a separate thread. Can be aborted using `abort()`. Can be blocking.
        If the duration is set to 0, the joints will move in their maximum speed without interpolation.

        Args:
            group (ControlGroup | list): The group of joints to move, or a list of joint indices.
            positions (numpy.ndarray[float]): target joint position in degrees.
            duration (float, optional): Time duration in seconds. If set to 0, the joints will move in their maximum speed without interpolation. Defaults to 0.0.
            blocking (bool, optional): If True, block until the move is completed. Defaults to False.
        """

        if not degrees:
            positions = numpy.rad2deg(positions)
        else:
            positions = numpy.asarray(positions)

        dest_pos = self.joint_positions.copy()

        if isinstance(group, ControlGroup):
            if positions.shape != (group.num_joints,):
                raise ValueError(f"Invalid joint position shape: {positions.shape}, expected: {(group.num_joints,)}")
            dest_pos[group.slice] = positions
        elif isinstance(group, list):
            if len(group) != len(positions):
                raise ValueError(f"Invalid joint position shape: {positions.shape}, expected: {(len(group),)}")
            dest_pos[group] = positions

        # TODO: automatic interpolation

        if self.is_moving:
            Logger().print_warning("Move already in progress, abort.")
            return

        if duration == 0:
            # with self._move_lock:
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

    def forward_kinematics(self, chain_names: list[str], q: numpy.ndarray | None = None):
        """Get the end effector pose of the specified chains.

        Args:
            chain_names (list[str]): The chains to get the end effector pose of. Available chain names: 'head', 'left_arm', 'right_arm'.
            q (numpy.ndarray, optional): The robot confiuration to do forward kinematics in. Defaults to None.

        Returns:
            list: The end effector pose of the specified chains.
        """

        res = self._call_service_wait(
            "forward_kinematics",
            value=Serde.pack({"chain_names": chain_names, "q": q}),
            timeout=0.1,
        )
        return res

    def inverse_kinematics(self, chain_names: list[str], targets: list[numpy.ndarray], dt: float, degrees=True):
        """Get the joint positions for the specified chains to reach the target pose.

        Args:
            chain_names (list[str]): The chains to get the joint positions for. Available chain names: 'head', 'left_arm', 'right_arm'.
            targets (list[numpy.ndarray]): The target poses.
            dt (float): The time step for the inverse kinematics.

        Returns:
            numpy.ndarray: The joint positions to reach the target pose.
        """

        res = self._call_service_wait(
            "inverse_kinematics",
            value=Serde.pack({"chain_names": chain_names, "targets": targets, "dt": dt}),
            timeout=0.1,
        )

        if res is not None and degrees:
            res = numpy.rad2deg(res)
        return res

    # ==============================================================================================================

    def get_transform(self, target_frame: str, source_frame: str, q: numpy.ndarray | None = None):
        """Get the transformation matrix between two frames in configuration `q`. If `q` is None, the current joint positions are used.

        Args:
            target_frame (str): Name of the frame to get the pose of.
            source_frame (str): Name of the frame to get the pose in.
            q (numpy.ndarray, optional): The robot confiuration to do forward kinematics in. Defaults to None.

        Returns:
            numpy.ndarray: The transformation matrix.
        """

        res = self._call_service_wait(
            f"tf/{source_frame}/{target_frame}",
            value=Serde.pack(q) if q is not None else None,
            timeout=1.0,
        )
        return numpy.array(res)

    def list_frames(self):
        """List all available frames."""

        res = self._call_service_wait(
            "tf/list",
            timeout=1.0,
        )
        return res

    # ==============================================================================================================

    def update_pos(self):
        """Update the joint positions command to the measured values.

        This is useful when the robot is moved manually and the joint positions are not updated.
        """
        # silently ignore if the robot is moving
        if self._move_lock.locked():
            return
        self._publish("position", self.states["joint"]["position"])

    def liveness_check(self) -> FunctionResult:
        """Check if the robot server is alive."""

        info = self._call_service_wait("robot/info", timeout=1.0)
        if info is None:
            Logger().print_error("Failed to connect to the robot server.  Please make sure the server is running.")
            return FunctionResult.FAIL

        return FunctionResult.SUCCESS

    def reboot(self):
        """Reboot the motors."""

        self._call_service_wait("control/reboot", timeout=10.0)
        self.update_pos()
        time.sleep(0.1)
        self.update_pos()
        time.sleep(0.1)
        self.update_pos()

    def set_enable(self, enable: bool):
        """Enable or disable the motors."""

        # self.update_pos()  # update position before enabling
        # time.sleep(0.1)

        cmd = "ON" if enable else "OFF"
        self._call_service_wait("control/enable", value=str(cmd), timeout=1.0)
        # time.sleep(0.1)

        # self.update_pos()  # update position after enabling
        # TODO: periodic update pose when disabled

    def set_home(self):
        """Get sensor offsets and save to `sensor_offset.json`"""

        self._call_service_wait("control/set_home", timeout=11.0)

    # ==============================================================================================================

    def play_traj(self, traj: list[numpy.ndarray], timestamps=None):
        """Play a trajectory in joint space."""

        # safely move to the start position
        Logger().print_info("Moving to start position...")
        self.move_joints(ControlGroup.ALL, traj[0], 2.0, blocking=True)
        Logger().print_info("Start position reached.")

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

    # ==============================================================================================================

    def abort(self):
        self._abort_event.set()
        self.update_pos()  # TODO:  test this

    def close(self):
        if self.server_connected:
            self.abort()
            # self.set_enable(False)
            time.sleep(0.1)

        super().close()

    # ==============================================================================================================


if __name__ == "__main__":
    import doctest

    doctest.testmod()
