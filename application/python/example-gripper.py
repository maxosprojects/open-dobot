#! /usr/bin/env python
'''
An example how to use gripper.

'''

from dobot import Dobot
import time

# The top Z to go to.
up = 180
# The bottom Z to go to.
down = 120
# Maximum speed in mm/s
speed = 700
# Acceleration in mm/s^2
acceleration = 400

# dobot = Dobot('/dev/tty.usbmodem1421', debug=True, fake=True)
# dobot = Dobot('COM4', debug=True)
dobot = Dobot('/dev/tty.usbmodem1421', debug=True)

# Enable calibration routine if you have a limit switch/photointerrupter installed on the arm.
# See example-switch.py for details.
# Take the tool to a safe height.

# dobot.MoveWithSpeed(260.0, 0.0, up, speed, acceleration)
# time.sleep(2)
# dobot.CalibrateJoint(1, dobot.freqToCmdVal(2000), dobot.freqToCmdVal(50), 1, 5, 1, 0)

dobot.Gripper(480)
# Take an object on the right and put to the left
dobot.MoveWithSpeed(260.0, 0.0, up, speed, acceleration, 0)
dobot.MoveWithSpeed(260.0, -70.0, up, speed, acceleration)
dobot.MoveWithSpeed(260.0, -70.0, down, speed, acceleration)
dobot.Gripper(208)
dobot.Wait(0.2)
dobot.MoveWithSpeed(260.0, -70.0, up, speed, acceleration)
dobot.MoveWithSpeed(260.0, 70.0, up, speed, acceleration, 500)
dobot.MoveWithSpeed(260.0, 70.0, down, speed, acceleration)
dobot.Gripper(480)
dobot.Wait(0.2)
dobot.MoveWithSpeed(260.0, 70.0, up, speed, acceleration)
dobot.Wait(0.5)
# Return to center
dobot.MoveWithSpeed(260.0, 0.0, up, speed, acceleration, 0)
