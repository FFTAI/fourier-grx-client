import math
import time

import numpy as np
import typer

from fourier_grx_client import ControlGroup, RobotClient


def test_server_impedance_control():
    client = RobotClient()
    client.set_enable(True)

    position_desired = np.zeros(20)  # deg

    # Simulation time parameters
    sim_dt = 0.01  # [s]
    sim_duration = 100.00  # [s]
    sim_steps = int(sim_duration / sim_dt)

    # please test the stability in advance when you control multi-joints
    for k in range(sim_steps):
        # current control
        # position_desired[0] = (15 * math.sin(0.01 * k - math.pi / 2) + 15)
        # position_desired[1] = 15 * math.sin(0.01 * k - math.pi / 2) + 15
        # position_desired[2] = (15 * math.sin(0.01 * k - math.pi / 2) + 15)# waist
        # position_desired[6] = -(15 * math.sin(0.01 * k - math.pi / 2) + 15)
        # position_desired[7] = 15 * math.sin(0.01 * k - math.pi / 2) + 15
        # position_desired[8] = -(15 * math.sin(0.01 * k - math.pi / 2) + 15)
        position_desired[9] = -(15 * math.sin(0.01 * k - math.pi / 2) + 15)
        # position_desired[10] = -(15 * math.sin(0.01 * k - math.pi / 2) + 15)# left arm
        # position_desired[13] = -(15 * math.sin(0.01 * k - math.pi / 2) + 15)
        # position_desired[14] = -(15 * math.sin(0.01 * k - math.pi / 2) + 15)
        # position_desired[15] = 15 * math.sin(0.01 * k - math.pi / 2) + 15
        # position_desired[16] = -(15 * math.sin(0.01 * k - math.pi / 2) + 15)
        # position_desired[17] = 15 * math.sin(0.01 * k - math.pi / 2) + 15# right arm

        # position control
        # position_desired[3] = -(15 * math.sin(0.025 * k))
        # position_desired[4] = -(15 * math.sin(0.025 * k))
        # position_desired[5] = -(15 * math.sin(0.025 * k))# head
        # position_desired[11] = -(15 * math.sin(0.025 * k))
        # position_desired[12] = -(15 * math.sin(0.025 * k))# left wrist
        # position_desired[18] = -(15 * math.sin(0.025 * k))
        # position_desired[19] = -(15 * math.sin(0.025 * k))# right wrist

        position_desired = np.deg2rad(position_desired)

        client.move_joints(ControlGroup.UPPER_EXTENDED, position_desired, gravity_compensation=True)
        # or
        # client.move_joints([23, 30],[position_desired[11], position_desired[18]], gravity_compensation=True)

        # When using Zenoh for communication, the maximum frequency can reach up to 120 Hz.
        # In contrast, when using wired transmission for communication, the maximum frequency can reach up to 200 Hz.
        time.sleep(1/120)
    print("progress ending......")
    client.disable()

if __name__ == "__main__":
    typer.run(test_server_impedance_control())

