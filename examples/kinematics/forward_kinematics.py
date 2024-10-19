import sys

import numpy as np
from fourier_grx_client import RobotClient
from loguru import logger

"""
This script demonstrates how to get the forward kinematics using the fourier_grx_client.
Before running this script, make sure to have the fourier_grx package installed and grx server running on the robot.
"""

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
        # Get the forward kinematics of the left arm using current joint positions
        chain = ["right_arm"]
        fk_output = client.forward_kinematics(chain)
        left_arm_fk = fk_output[0]
        logger.info(f"Left arm forward kinematics: {left_arm_fk}")
        
        # Get the forward kinematics of the left arm using specified joint positions
        q = client.joint_positions
        fk_output = client.forward_kinematics(chain, q)
        left_arm_fk = fk_output[0]
        logger.info(f"Left arm forward kinematics: {left_arm_fk}")
    
        # Convert the rotation matrix to xyz-quaternion
        left_arm_cartesian = rotation_matrix_to_xyz_quat(left_arm_fk)
        logger.info(f"Left arm cartesian: {left_arm_cartesian}")

        # Close the connection to the robot server
        client.close()
        return True
    except Exception as e:
        logger.error(f"Error occured while getting forward kinematics: {e}")
        return False


if __name__ == "__main__":
    if not main():
        sys.exit(1)
