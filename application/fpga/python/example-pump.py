#! /usr/bin/env python
'''
Take an object using sucker pump and put it in a different place.

'''

from DobotSDK import Dobot
import time

dobot = Dobot('/dev/tty.usbmodem1421', debug=True)

# Enable calibration routine if you have a limit switch/photointerrupter installed on the arm.
# See example-switch.py for details.
# Move both arms to approximately 45 degrees.
# dobot.moveWithSpeed(260.0, 0.0, 85, 700)
# time.sleep(2)
# dobot.CalibrateJoint(1, dobot.freqToCmdVal(2000), dobot.freqToCmdVal(50), 1, 5, 1, 0)

def repeatUntilQueued(on):
	ret = (0,0)
	while not ret[0] or not ret[1]:
		ret = dobot.PumpOn(on)
	ret = (0,0)
	while not ret[0] or not ret[1]:
		ret = dobot.ValveOn(on)

dobot.MoveWithSpeed(200.0, -50.0, 180.0, 100)
dobot.MoveWithSpeed(200.0, -50.0, 120.0, 100)
repeatUntilQueued(True)
dobot.MoveWithSpeed(200.0, -50.0, 180.0, 100)
dobot.MoveWithSpeed(200.0, 50.0, 180.0, 100)
dobot.MoveWithSpeed(200.0, 50.0, 120.0, 100)
repeatUntilQueued(False)
dobot.MoveWithSpeed(200.0, 50.0, 180.0, 100)
dobot.MoveWithSpeed(200.0, 0.0, 180.0, 100)
