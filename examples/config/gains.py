import sys

from fourier_grx_client import RobotClient
from loguru import logger

"""
This script demonstrates how to get and set gains using the fourier_grx_client.
Before running this script, make sure to have the fourier_grx package installed and grx server running on the robot.
"""


def main():
    # Create a RobotClient object and connect to the robot server
    client = RobotClient(namespace="gr/my_awesome_robot", server_ip="localhost")

    try:
        # Get the gains of the robot joints
        joint_gains = client.get_gains()
        logger.info(f"Joint gains: {joint_gains}")

        # get the gains of the left arm joints
        left_arm_kp = joint_gains["pd_control_kp"][18:25]
        left_arm_kd = joint_gains["pd_control_kd"][18:25]
        logger.info(f"Left arm kp: {left_arm_kp}")
        logger.info(f"Left arm kd: {left_arm_kd}")

        # Set the gains of the left arm joints
        client.set_gains(pd_control_kp=joint_gains["pd_control_kp"], pd_control_kd=joint_gains["pd_control_kd"])
        logger.success("Gains set successfully")

        # Close the connection to the robot server
        client.close()
        return True
    except Exception as e:
        logger.error(f"Error occurred: {e}")
        return False


if __name__ == "__main__":
    if not main():
        sys.exit(1)
