#! /usr/bin/env python
'''
Laser going in a straight line.

!!!!! TAKE ALL POSSIBLE PRECAUTIONS TO PREVENT FIRE !!!!!
!!!!! BE READY TO DISCONNECT THE ARM FROM POWER SOURCE AT ANY TIME !!!!!

'''

from DobotSDK import Dobot
import time

dobot = Dobot('/dev/tty.usbmodem1421', debug=True)

# Enable CalibrateJoint to calibrate base if you have a limit switch/photointerrupter mounted and connected.
# dobot.CalibrateJoint(1, dobot.freqToCmdVal(1000), dobot.freqToCmdVal(50), 1, 5, 1, 0)
dobot.moveWithSpeed(200.0, -50.0, 100.0, 50)
time.sleep(2)
dobot.LaserOn(True)
dobot.moveWithSpeed(200.0, 50.0, 100.0, 0.75)
time.sleep(4)
dobot.LaserOn(False)
