#! /usr/bin/env python
'''
Laser going in a straight line.

!!!!! TAKE ALL POSSIBLE PRECAUTIONS TO PREVENT FIRE !!!!!
!!!!! BE READY TO DISCONNECT THE ARM FROM POWER SOURCE AT ANY TIME !!!!!

'''

from dobot import Dobot
import time

# dobot = Dobot('COM4', debug=True)
dobot = Dobot('/dev/tty.usbmodem1421', debug=True)

# Enable calibration routine if you have a limit switch/photointerrupter installed on the arm.
# See example-switch.py for details.
# Move both arms to approximately 45 degrees.
# dobot.MoveWithSpeed(260.0, 0.0, 85, 700)
# time.sleep(2)
# dobot.CalibrateJoint(1, dobot.freqToCmdVal(2000), dobot.freqToCmdVal(50), 1, 5, 1, 0)

def repeatUntilQueued(on):
	ret = (0,0)
	while not ret[0] or not ret[1]:
		ret = dobot.LaserOn(on)

dobot.MoveWithSpeed(200.0, -50.0, 100.0, 50)
repeatUntilQueued(True)
dobot.MoveWithSpeed(200.0, 50.0, 100.0, 0.75)
repeatUntilQueued(False)
