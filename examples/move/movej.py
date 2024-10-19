from fourier_grx_client import RobotClient, ControlGroup
from loguru import logger
import time
import sys

'''
This script demonstrates how to apply joint space control using the fourier_grx_client.
Before running this script, make sure to have the fourier_grx package installed and grx server running on the robot.
'''

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
        
        # Prepare the target positions for the arms
        sides = ['left', 'right']
        target_position_left = [-0.23, 0.2, 0.22, 0.1, 0.8, 0.0, 0.0]
        target_position_right = [0.01, -0.07, -0.05, -0.9, 0.02, 0.0, 0.0]
        target_positions = [target_position_left, target_position_right]

        # Move the joints of the arms to target positions using movej
        traj=client.movej(sides, target_positions)
        logger.success("Arm joints moved to target positions")
        time.sleep(1)

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
    



