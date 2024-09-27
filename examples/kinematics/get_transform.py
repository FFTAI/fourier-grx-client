import sys

from fourier_grx_client import RobotClient
from loguru import logger

"""
This script demonstrates how to get the transformation matrix using the fourier_grx_client.
Before running this script, make sure to have the fourier_grx package installed and grx server running on the robot.
"""


def main():
    # Create a RobotClient object and connect to the robot server
    client = RobotClient(namespace="gr/my_awesome_robot", server_ip="localhost")

    try:
        # List available frames
        frames = client.list_frames()
        logger.info(f"Available frames: {frames}")

        # Get the transformation matrix from the torso to the left arm
        target_frame = "left_end_effector_link"
        source_frame = "torso_link"
        transform = client.get_transform(target_frame, source_frame)
        logger.info(f"Transformation matrix from {source_frame} to {target_frame}:\n{transform}")

        # Close the connection to the robot server
        client.close()
        return True
    except Exception as e:
        logger.error(f"Error occured while getting transform: {e}")
        return False


if __name__ == "__main__":
    if not main():
        sys.exit(1)
