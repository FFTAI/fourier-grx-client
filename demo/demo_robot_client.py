import threading
import time

import msgpack_numpy as m
import numpy as np
import zenoh

from fourier_grx_client import ControlGroup, RobotClient
from rich.console import Console
from rich.pretty import pprint
from rich.prompt import Confirm, Prompt
from rich.table import Table

m.patch()

zenoh.init_logger()
console = Console()
log = console.log
print = console.print

FREQUENCY = 150

np.set_printoptions(precision=2, suppress=True)


def task_enable(robot_client: RobotClient):
    robot_client.set_enable(True)


def task_disable(robot_client: RobotClient):
    robot_client.set_enable(False)


def task_set_home(robot_client: RobotClient):
    robot_client.set_home()


def task_reboot(robot_client: RobotClient):
    robot_client.reboot()


def task_move_left_arm_to_default(robot_client: RobotClient):
    robot_client.set_enable(True)
    time.sleep(0.5)
    robot_client.move_joints(
        ControlGroup.LEFT_ARM,
        robot_client.default_group_positions[ControlGroup.LEFT_ARM],
        2.0,
        blocking=False,
    )


def task_move_to_default(robot_client: RobotClient):
    robot_client.set_enable(True)
    time.sleep(0.5)
    robot_client.move_joints(
        ControlGroup.UPPER_EXTENDED,
        robot_client.default_group_positions[ControlGroup.UPPER_EXTENDED],
        2.0,
        blocking=False,
    )


def task_abort(robot_client: RobotClient):
    robot_client.abort()


def task_print_states(robot_client: RobotClient):
    table = Table("Type", "Data", title="Current :robot: state")
    for sensor_type, sensor_data in robot_client.states.items():
        for sensor_name, sensor_reading in sensor_data.items():
            print(sensor_type + "/" + sensor_name, sensor_reading.tolist())
            table.add_row(
                sensor_type + "/" + sensor_name,
                str(np.round(sensor_reading, 3)),
            )
    print(table)


def task_list_frames(robot_client: RobotClient):
    frames = robot_client.list_frames()
    print(frames)


def task_get_transform(robot_client: RobotClient):
    q = robot_client.joint_positions.copy()
    q[-3] += 20.0
    # q = None
    transform = robot_client.get_transform("r_wrist_roll", "base", q=q)
    print(f"Transform from base to r_wrist_roll: {transform} in configuration q={q}")


def record(robot_client: RobotClient):
    traj = []
    robot_client.set_enable(False)

    time.sleep(1)

    reply = Prompt.ask("Move to start position and press enter")
    if reply == "":
        robot_client.update_pos()
        time.sleep(0.1)
        robot_client.set_enable(True)
        time.sleep(1)
        for sensor_type, sensor_data in robot_client.states.items():
            for sensor_name, sensor_reading in sensor_data.items():
                if sensor_type == "joint":
                    print(sensor_type + "/" + sensor_name, sensor_reading.tolist())
    else:
        return
    time.sleep(0.5)
    reply = Confirm.ask("Start recording?")

    if not reply:
        return

    # robot_client.update_pos()
    robot_client.set_enable(False)
    time.sleep(1)
    event = threading.Event()

    def inner_task():
        while not event.is_set():
            robot_client.loop_manager.start()
            traj.append(robot_client.joint_positions.copy())
            robot_client.loop_manager.end()
            robot_client.loop_manager.sleep()

    thread = threading.Thread(target=inner_task)
    thread.daemon = True
    thread.start()

    reply = Prompt.ask("Press enter to stop recording")
    if reply == "":
        event.set()
        thread.join()

        robot_client.update_pos()
        time.sleep(0.1)
        robot_client.set_enable(True)
        np.save("record.npy", traj)
        return traj


def task_record(robot_client: RobotClient):
    traj = record(robot_client)
    pprint(traj)


def play(recorded_traj: list[np.ndarray], robot_client: RobotClient):
    robot_client.set_enable(True)
    time.sleep(1)

    first = recorded_traj[0]
    robot_client.move_joints(ControlGroup.ALL, first, 2.0, blocking=True)
    for pos in recorded_traj[1:]:
        robot_client.move_joints(ControlGroup.ALL, pos, duration=0.0)
        time.sleep(1 / FREQUENCY)
    time.sleep(1)
    robot_client.set_enable(False)


def task_play(robot_client: RobotClient):
    rec = np.load("record.npy", allow_pickle=True)
    play(rec, robot_client)


def task_set_gains(robot_client: RobotClient):
    kp = [0.1] * 32
    kd = [0.01] * 32
    new_gains = robot_client.set_gains(pd_control_kp=kp, pd_control_kd=kd)
    print(new_gains)


def task_exit(robot_client: RobotClient):
    import sys

    robot_client.close()
    sys.exit(0)


if __name__ == "__main__":
    robot_client = \
        RobotClient(
            namespace="gr/robot_fftai",  # check robot server namespace from server start log
            freq=FREQUENCY,
            server_ip="192.168.3.68",  # change to use the IP address of your robot
        )

    time.sleep(0.5)

    while True:
        task = Prompt.ask(
            "What do you want the :robot: to do?",
            choices=[
                "enable",
                "disable",
                "set_home",
                "set_gains",
                "reboot",
                "print_states",
                "move_to_default",
                "record",
                "play",
                "abort",
                "list_frames",
                "get_transform",
                "exit",
            ],
        )
        if task == "enable":
            task_enable(robot_client)
        elif task == "disable":
            task_disable(robot_client)
        elif task == "set_home":
            task_set_home(robot_client)
        elif task == "set_gains":
            task_set_gains(robot_client)
        elif task == "reboot":
            task_reboot(robot_client)
        elif task == "move_to_default":
            task_move_to_default(robot_client)
        elif task == "abort":
            task_abort(robot_client)
        elif task == "print_states":
            task_print_states(robot_client)
        elif task == "record":
            task_record(robot_client)
        elif task == "play":
            task_play(robot_client)
        elif task == "exit":
            task_exit(robot_client)
        elif task == "list_frames":
            task_list_frames(robot_client)
        elif task == "get_transform":
            task_get_transform(robot_client)

        time.sleep(0.5)

    # robot_client.spin()
    # time.sleep(1)
    # robot_client.set_enable(False)
