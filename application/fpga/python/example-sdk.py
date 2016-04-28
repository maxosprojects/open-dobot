#! /usr/bin/env python
'''
Simple demostration of open-dobot. High-level control via SDK.

It is assumed that no end effectors are installed on the arm (no gripper or sucker, etc.)
as end effector would crash into the desk Dobot stands on when folowing the examples below.
If you want to run the following examples with an end effector you would need to adjust all
coordinates in the commands correspondingly and make sure they are still in arm's in reachable area.

Refer to SDK to find the expected initial arm configuration.

Each move is split into a series of commands to send via driver to FPGA. Each command is
executed for 20ms. Dobot, thus, executes 50 commands per second.

The commands are queued in Arduino, hence when you stop this example dobot will continue
to execute until the queue (200 commands) is empty.

'''

from DobotSDK import Dobot
import time

# The top Z to go to.
up = 50
# The bottom Z to go to.
down = 39
# Maximum speed in mm/s
speed = 700
# Acceleration in mm/s^2
acceleration = 400

# dobot = Dobot('/dev/tty.usbmodem1421', debug=True, fake=True)
dobot = Dobot('/dev/tty.usbmodem1421', debug=True)
# dobot = Dobot('/dev/tty.BT4-SPP-SerialPort', debug=True, timeout=0.3)

# Enable calibration routine if you have a limit switch/photointerrupter installed on the arm.
# See example-switch.py for details.
# Move both arms to approximately 45 degrees.
# dobot.MoveWithSpeed(260.0, 0.0, 85, 700, acceleration)
# time.sleep(2)
# dobot.CalibrateJoint(1, dobot.freqToCmdVal(2000), dobot.freqToCmdVal(50), 1, 5, 1, 0)

# Line
# dobot.MoveWithSpeed(200.0, 80.0, up, speed, acceleration)
# dobot.MoveWithSpeed(200.0, 80.0, down, speed, acceleration)
# dobot.MoveWithSpeed(200.0, -90.0, down, speed, acceleration)
# dobot.MoveWithSpeed(200.0, -90.0, up, speed, acceleration)

# dobot.MoveWithSpeed(200.0, -90.0, down, speed, acceleration)
# dobot.MoveWithSpeed(200.0, 80.0, down, speed, acceleration)
# dobot.MoveWithSpeed(200.0, 80.0, up, speed, acceleration)
# dobot.MoveWithSpeed(200.0, -90.0, up, speed, acceleration)

# Rectangle with zig-zag inside
dobot.MoveWithSpeed(170.0, -90.0, up, speed, acceleration)
dobot.MoveWithSpeed(170.0, -90.0, down, speed, acceleration)
dobot.MoveWithSpeed(170.0, 80.0, down, speed, acceleration)
dobot.MoveWithSpeed(230.0, 80.0, down, speed, acceleration)
dobot.MoveWithSpeed(230.0, -90.0, down, speed, acceleration)
dobot.MoveWithSpeed(170.0, -90.0, down, speed, acceleration)
x = 230
y = 0
for y in range(-90, 81, 5):
	if x == 170:
		x = 230
	else:
		x = 170
	dobot.MoveWithSpeed(x, y, down, speed, acceleration)

dobot.MoveWithSpeed(x, y, up, speed, acceleration)
dobot.MoveWithSpeed(200.0, -90.0, up, speed, acceleration)

# Jog
# while True:
# 	dobot.MoveWithSpeed(200.0, 80.0, up, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, 80.0, down, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, 80.0, up, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, 80.0, 200, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, -80.0, up, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, -80.0, down, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, -80.0, up, speed, acceleration)

# Dashed line
# while True:
# 	dobot.MoveWithSpeed(200.0, 80.0, up, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, 80.0, down, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, 70.0, down, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, 70.0, up, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, 40.0, up, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, 40.0, down, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, 30.0, down, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, 30.0, up, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, 0.0, up, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, 0.0, down, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, -10.0, down, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, -10.0, up, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, -40.0, up, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, -40.0, down, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, -50.0, down, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, -50.0, up, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, -80.0, up, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, -80.0, down, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, -90.0, down, speed, acceleration)
# 	dobot.MoveWithSpeed(200.0, -90.0, up, speed, acceleration)
