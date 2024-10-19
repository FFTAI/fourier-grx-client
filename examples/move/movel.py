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

        # Move the joints of arms to home position
        client.move_joints(ControlGroup.UPPER, [0.0]*14, duration=1.0)
        logger.success("Arm joints returned to home position")
        time.sleep(1)

        # Prepare the target cartesian positions
        sides = ["left", "right"]
        target_position_array_left = np.array([[ 0.43, 0.6, -0.68, 0.22],
                                        [-0.89, 0.38, -0.23, 0.32],
                                        [0.12, 0.7, 0.7, 0.01],
                                        [0.0, 0.0, 0.0, 1.0]])
        target_position_array_right = np.array([[ 0.9, -0.16, -0.41, 0.07],
                                        [0.36, 0.82, 0.46, -0.34],
                                        [0.26, -0.56, 0.79, -0.05],
                                        [0.0, 0.0, 0.0, 1.0]])
        target_pose = [target_position_array_left, target_position_array_right]

        # Move the arms to target cartesian positions using movel 
        client.movel(sides, target_pose)
        logger.success("Arm joints moved to target cartesian positions")
        time.sleep(1)

        # Move the joints of arms to home position
        client.move_joints(ControlGroup.UPPER, [0.0]*14, duration=1.0)
        logger.success("Arm joints returned to home position")
        time.sleep(1)

        # Create target cartesian positions for the arms using rotation matrix
        target_position_left = np.array(rotation_matrix_to_xyz_quat(target_position_array_left))
        target_position_right = np.array(rotation_matrix_to_xyz_quat(target_position_array_right))
        target_pose = [target_position_left, target_position_right]
        
        # Move the arms to target cartesian positions using movel
        client.movel(sides, target_pose)
        time.sleep(1)
        logger.success("Arm joints moved to target cartesian positions")

        # Move the joints of arms to home position
        client.move_joints(ControlGroup.UPPER, [0.0]*14, duration=1.0)
        logger.success("Arm joints returned to home position")
        time.sleep(1)

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
    



