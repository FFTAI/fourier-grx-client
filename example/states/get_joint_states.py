from fourier_grx_client import RobotClient, ControlGroup
from loguru import logger
import sys

'''
This script demonstrates how to get the joint states using the fourier_grx_client.
Before running this script, make sure to have the fourier_grx package installed and grx server running on the robot.
'''
def main():
    # Create a RobotClient object and connect to the robot server
    client = RobotClient(namespace="gr/my_awesome_robot", server_ip="localhost")

    try:
        # Get the joint positions of the right arm using ControlGroup 
        right_arm_positions = client.get_group_position(ControlGroup.RIGHT_ARM)
        logger.info(f"Right arm joint positions: {right_arm_positions}")
        # Get the joint positions of the left arm using joint index
        left_arm_positions = client.joint_positions[18:25]
        logger.info(f"Left arm joint positions: {left_arm_positions}")
        
        # Get the joint velocities of the right arm using ControlGroup 
        right_arm_velocities = client.get_group_velocity(ControlGroup.RIGHT_ARM)
        logger.info(f"Right arm joint velocities: {right_arm_velocities}")
        # Get the joint velocities of the left arm using joint index
        left_arm_velocities = client.joint_velocity[18:25]
        logger.info(f"Left arm joint velocities: {left_arm_velocities}")
        
        # Get the joint currents of the right arm using ControlGroup 
        right_arm_currents = client.get_group_current(ControlGroup.RIGHT_ARM)
        logger.info(f"Right arm joint currents: {right_arm_currents}")
        # Get the joint currents of the left arm using joint index
        left_arm_currents = client.joint_current[18:25]
        logger.info(f"Left arm joint currents: {left_arm_currents}")

        # Get the joint efforts of the right arm using ControlGroup 
        right_arm_efforts = client.get_group_effort(ControlGroup.RIGHT_ARM)
        logger.info(f"Right arm joint efforts: {right_arm_efforts}")
        # Get the joint efforts of the left arm using joint index
        left_arm_efforts = client.joint_effort[18:25]
        logger.info(f"Left arm joint efforts: {left_arm_efforts}")
        
        # Close the connection to the robot server
        client.close()
        return True
    except Exception as e:
        logger.error(f"Error occured while getting joint states: {e}")
        return False

if __name__ == '__main__':
    if not main():
        sys.exit(1)
    



