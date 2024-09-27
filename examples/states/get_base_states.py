import sys

from fourier_grx_client import RobotClient
from loguru import logger

"""
This script demonstrates how to get the base estimator states using the fourier_grx_client.
Before running this script, make sure to have the fourier_grx package installed and grx server running on the robot.
"""


def main():
    # Create a RobotClient object and connect to the robot server
    client = RobotClient(namespace="gr/my_awesome_robot", server_ip="localhost")

    try:
        # Get the base estimator postion
        base_pose = client.base_pose
        logger.info(f"Base position: {base_pose}")

        # Get the base estimator velocity
        base_velocity = client.base_velocity
        logger.info(f"Base velocity: {base_velocity}")

        # Close the connection to the robot server
        client.close()
        return True
    except Exception as e:
        logger.error(f"Error occured while getting base states: {e}")
        return False


if __name__ == "__main__":
    if not main():
        sys.exit(1)
