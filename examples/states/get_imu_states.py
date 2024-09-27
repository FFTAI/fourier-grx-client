import sys

from fourier_grx_client import RobotClient
from loguru import logger

"""
This script demonstrates how to get the imu states using the fourier_grx_client.
Before running this script, make sure to have the fourier_grx package installed and grx server running on the robot.
"""


def main():
    # Create a RobotClient object and connect to the robot server
    client = RobotClient(namespace="gr/my_awesome_robot", server_ip="localhost")

    try:
        # Get the imu quaternion data
        imu_quaternion = client.imu_quaternion
        logger.info(f"imu quaternion: {imu_quaternion}")

        # Get the imu euler angle data
        imu_angles = client.imu_angles
        logger.info(f"imu angles: {imu_angles}")

        # Get the imu angular velocity data
        imu_angular_velocity = client.imu_angular_velocity
        logger.info(f"imu angular velocity: {imu_angular_velocity}")

        # Get the imu linear acceleration data
        imu_linear_acceleration = client.imu_linear_acceleration
        logger.info(f"imu linear acceleration: {imu_linear_acceleration}")

        # Close the connection to the robot server
        client.close()
        return True
    except Exception as e:
        logger.error(f"Error occured while getting imu states: {e}")
        return False


if __name__ == "__main__":
    if not main():
        sys.exit(1)
