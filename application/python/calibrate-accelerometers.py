#! /usr/bin/env python

"""
open-dobot Accelerometers Calibration tool.

This tool continuously reports accelerometers and angles from those.

Use this tool to find offsets for your accelerometers.

Follow the procedure below to enable accelerometers reporting mode on FPGA.
No action required on RAMPS as GY-521 accelerometers can be read at any time there.
1. Turn off power on the arm and disconnect USB cable
2. Remove accelerometers from the arm and put them on a flat surface that has no inclination
3. Connect USB cable
4. Enable accelerometers reporting mode:
   4.1. Press and hold the "Sensor Calibration" button on FPGA version or ground pin D23 on AUX-4 on RAMPS version
   4.2. Press and release the "Reset" button
   4.3. Start this tool (still holding "Sensor Calibration" button on FPGA version or keeping pin D23 grounded on RAMPS)
   4.4. Wait for the accelerometer data to start flowing on your console/whatever_you_use_to_start_this_tool
   4.5. Release the "Sensor Calibration" button
5. Gently push down the accelerometers so that they are evenly on the surface. Don't touch any contacts/leads.
	You can push them one by one, not necessary to push both at the same time
6. Note the "Raw" data from accelerometers reported on the console. Those are your accelerometers' offsets
7. Turn off power on the arm, disconnect USB cable, mount accelerometers back onto the arm 

Author: maxosprojects (March 18 2016)
Additional Authors: <put your name here>

Version: 1.2.2

License: MIT
"""
import math

from dobot import DobotDriver
from dobot import DobotKinematics

# driver = DobotDriver('COM4')
driver = DobotDriver('/dev/tty.usbmodem1421')
driver.Open()
# driver.Open(timeout=0.3)
kinematics = DobotKinematics()

# Offsets must be found using this tool for your Dobot once
# (rear arm, frontarm)
offsets = (1024, 1024)

def toEndEffectorHeight(rear, front):
	ret = kinematics.coordinatesFromAngles(0, rear, front)
	return ret[2]

while True:
	ret = driver.GetAccelerometers()
	if ret[0]:
		if driver.isFpga():
			print("Rear arm: {0:10f} | Front arm: {1:10f} | End effector height: {2:10f} | Raw rear arm: {3:4d} | Raw front arm: {4:4d}".format(\
				driver.accelToAngle(ret[1], offsets[0]), driver.accelToAngle(ret[4], offsets[1]),\
				toEndEffectorHeight(driver.accelToRadians(ret[1], offsets[0]), driver.accelToRadians(ret[4], offsets[1])),\
				ret[1], ret[4]))
		else:
			print("Rear arm: {0:6.2f} | Front arm: {1:6.2f} | End effector height: {2:7.2f} | Raw rear arm: {3:6d} {4:6d} {5:6d} | Raw front arm: {6:6d} {7:6d} {8:6d}".format(\
				driver.accel3DXToAngle(ret[1], ret[2], ret[3]), -driver.accel3DXToAngle(ret[4], ret[5], ret[6]),\
				toEndEffectorHeight(driver.accel3DXToRadians(ret[1], ret[2], ret[3]), -driver.accel3DXToRadians(ret[4], ret[5], ret[6])),\
				ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]))
	else:
		print('Error occurred reading data')

