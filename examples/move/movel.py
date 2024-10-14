from fourier_grx_client import RobotClient, ControlGroup
from loguru import logger
import time
import sys
import numpy as np

'''
This script demonstrates how to apply cartesian space control using the fourier_grx_client.
Before running this script, make sure to have the fourier_grx package installed and grx server running on the robot.
'''

def rotation_matrix_to_xyz_quat(rotation_matrix: np.ndarray):
    assert rotation_matrix.shape == (4, 4), "Invalid rotation matrix shape!"

    translation = rotation_matrix[:3, 3]
    R = rotation_matrix[:3, :3]

    qw = np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
    qx = (R[2, 1] - R[1, 2]) / (4 * qw)
    qy = (R[0, 2] - R[2, 0]) / (4 * qw)
    qz = (R[1, 0] - R[0, 1]) / (4 * qw)

    return translation.tolist() + [qx, qy, qz, qw]

def main():
    # Create a RobotClient object and connect to the robot server
    client = RobotClient(namespace="gr/my_awesome_robot", server_ip="localhost")

    try:
        # Enable the robot motors
        client.enable()
        logger.info("Motors enabled")
        time.sleep(1)

        # Move the joints of the left arm to home position
        client.move_joints(ControlGroup.LEFT_ARM, [0.0]*7, duration=1.0)
        logger.success("Left arm joints returned to home position")
        time.sleep(1)

        # Move the left arm to a target cartesian position using movel 
        target_position_array = np.array([[ 0.079, -0.067, -0.995,  0.292],
        [-0.995,  0.056, -0.082,  0.124],
        [ 0.062,  0.996, -0.063,  0.122],
        [ 0.   ,  0.   ,  0.   ,  1.   ]])
        client.movel(["left"],[target_position_array])
        logger.success("Left arm joints moved to target cartesian position")
        time.sleep(1)

        # Move the joints of the left arm to home position
        client.move_joints(ControlGroup.LEFT_ARM, [0.0]*7, duration=1.0)
        logger.success("Left arm joints returned to home position")
        time.sleep(1)

        # Move the left arm to a target cartesian position using movel
        target_position = np.array(rotation_matrix_to_xyz_quat(target_position_array))
        client.movel(["left"],[target_position])
        time.sleep(1)
        logger.success("Left arm joints moved to target cartesian position")

        # Disable the robot motors
        client.disable()
        logger.info("Motors disabled")

        # Close the connection to the robot server
        client.close()
        return True
    except Exception as e:
        logger.error(f"Error occured: {e}")
        return False

if __name__ == '__main__':
    if not main():
        sys.exit(1)
    



