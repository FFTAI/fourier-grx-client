import time

from fourier_dhx.sdk.DexHand import DexHand

# Ensure that the server is running on the robot before executing this script.

# Instantiate the hand object
# For left hand use IP: 192.168.137.39
# For right hand use IP: 192.168.137.19

_back, _stop = [-200, -200, -200, -200, -200, -200], [0, 0, 0, 0, 0, 0]
HAND_IP = "192.168.137.39"  # Change to the appropriate IP address
hand = DexHand(HAND_IP)
hand.set_pwm(_back)
hand.calibration()

time.sleep(1.0)  # Wait for the device to initialize

angle = [10.0, 10.0, 8.0, 5.0, 5.0, 5.0]
hand.set_angle(0, angle)
time.sleep(1)

# verify
pos = hand.get_pos()
print(pos)


hand.set_pwm(_back)
time.sleep(2)
hand.set_pwm(_stop)
time.sleep(0.01)
