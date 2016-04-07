"""
open-dobot SDK.

SDK providing high-level functions to control Dobot via the driver to open firmware, which, in turn, controls Dobot FPGA.
Abstracts specifics of commands sent to FPGA.
Find firmware and driver at https://github.com/maxosprojects/open-dobot

It is assumed that upon SDK initialization the arms are beetween 0 and 90 degrees - beetween their normal
horizontal and vertical positions.
Upon initialization accelerometers are read to figure out current arms' configuration. Accelerometers get confused
when rear arm leans backwards from the dobot base or when forearm bends towards the base.
Also, Inverse Kinematics at the moment don't account for when forearm is looking up (higher than it's
normal horizontal position). So be gentle and give dobot some feasible initial configuration in case it happened
to be beyond the mentioned limits.
Refer to docs/images/ to find more about reference frame, arm names and more.

SDK keeps track of the current end effector pose, thus in case the arm slips or motors are disabled while
in move (with the "Laser Adjustment" button) it has to be re-initialized and SDK re-initialized.

Author: maxosprojects (March 18 2016)
Additional Authors: <put your name here>

Version: 0.5.0

License: MIT
"""

import serial
import threading
import time
from serial import SerialException
from DobotDriver import DobotDriver
from DobotInverseKinematics import DobotInverseKinematics
import timeit
import math
import matplotlib.pyplot as plt

piHalf = math.pi / 2.0
piTwo = math.pi * 2.0
piThreeFourths = math.pi * 3.0 / 4.0
# calibration-tool.py for details
accelOffsets = (1024, 1024)

# The NEMA 17 stepper motors that Dobot uses are 200 steps per revolution.
stepperMotorStepsPerRevolution = 200.0
# FPGA board has all stepper drivers' stepping pins set to microstepping.
baseMicrosteppingMultiplier = 16.0
rearArmMicrosteppingMultiplier = 16.0
foreArmMicrosteppingMultiplier = 16.0
# The NEMA 17 stepper motors Dobot uses are connected to a planetary gearbox, the black cylinders
# with 10:1 reduction ratio
stepperPlanetaryGearBoxMultiplier = 10.0

# calculate the actual number of steps it takes for each stepper motor to rotate 360 degrees
baseActualStepsPerRevolution = stepperMotorStepsPerRevolution * baseMicrosteppingMultiplier * stepperPlanetaryGearBoxMultiplier
rearArmActualStepsPerRevolution = stepperMotorStepsPerRevolution * rearArmMicrosteppingMultiplier * stepperPlanetaryGearBoxMultiplier
foreArmActualStepsPerRevolution = stepperMotorStepsPerRevolution * foreArmMicrosteppingMultiplier * stepperPlanetaryGearBoxMultiplier

class Dobot:
	def __init__(self, port, rate=115200, timeout=0.025, debug=False, fake=False):
		self._debugOn = debug
		self._fake = fake
		self._driver = DobotDriver(port, rate)
		if not fake:
			self._driver.Open(timeout)
		self._ik = DobotInverseKinematics(debug=debug)
		# Initialize arms current configuration from accelerometers
		if fake:
			self._baseSteps = long(0)
			self._rearSteps = long(0)
			self._foreSteps = long(0)
		else:
			# self._baseSteps = long(0)
			# self._rearSteps = long(0)
			# self._foreSteps = long(0)
			accels = self._driver.GetAccelerometers()
			accelRear = accels[1]
			accelFore = accels[2]
			rearAngle = math.pi / 2 - self._driver.accelToRadians(accelRear, accelOffsets[0])
			foreAngle = self._driver.accelToRadians(accelFore, accelOffsets[1])
			self._baseSteps = long(0)
			self._rearSteps = long((rearAngle / math.pi / 2.0) * rearArmActualStepsPerRevolution + 0.5)
			self._foreSteps = long((foreAngle / math.pi / 2.0) * foreArmActualStepsPerRevolution + 0.5)
			self._driver.SetCounters(self._baseSteps, self._rearSteps, self._foreSteps)
			print "Initializing with steps:", self._baseSteps, self._rearSteps, self._foreSteps
			print "Reading back what was set:", self._driver.GetCounters()
			print "--=========--"


	def _debug(self, *args):
		if self._debugOn:
			# Since "print" is not a function the expansion (*) cannot be used
			# as it is not an operator. So this is a workaround.
			for arg in args:
				print arg,
			print

	def _moveArmToAngles(self, baseAngle, rearArmAngle, foreArmAngle, duration):
		self._baseAngle = baseAngle
		self._rearAngle = rearArmAngle
		self._foreAngle = foreArmAngle
		dur = float(duration)

		# baseStepLocation = long((baseAngle / 360.0) * baseActualStepsPerRevolution + 0.5)
		# rearArmStepLocation = long((abs(rearArmAngle) / 360.0) * rearArmActualStepsPerRevolution + 0.5)
		# foreArmStepLocation = long((abs(foreArmAngle) / 360.0) * foreArmActualStepsPerRevolution + 0.5)
		baseStepLocation = long(baseAngle * baseActualStepsPerRevolution / piTwo)
		rearArmStepLocation = long(rearArmAngle * rearArmActualStepsPerRevolution / piTwo)
		foreArmStepLocation = long(foreArmAngle * foreArmActualStepsPerRevolution / piTwo)

		self._debug("Base Step Location", baseStepLocation)
		self._debug("Rear Arm Step Location", rearArmStepLocation)
		self._debug("Forearm Step Location", foreArmStepLocation)

		baseDiff = baseStepLocation - self._baseSteps
		rearDiff = rearArmStepLocation - self._rearSteps
		foreDiff = foreArmStepLocation - self._foreSteps
		self._debug('baseDiff', baseDiff)
		self._debug('rearDiff', rearDiff)
		self._debug('foreDiff', foreDiff)

		self._baseSteps = baseStepLocation
		self._rearSteps = rearArmStepLocation
		self._foreSteps = foreArmStepLocation

		baseDir = 1
		rearDir = 1
		foreDir = 1

		if (baseDiff < 1):
			baseDir = 0
		if (rearDiff < 1):
			rearDir = 0
		if (foreDiff > 1):
			foreDir = 0

		baseSliced = self._sliceStepsToValues(abs(baseDiff), dur)
		rearSliced = self._sliceStepsToValues(abs(rearDiff), dur)
		foreSliced = self._sliceStepsToValues(abs(foreDiff), dur)

		for base, rear, fore in zip(baseSliced, rearSliced, foreSliced):
			ret = [0, 0]
			# If ret[0] == 0 then command timed out or crc failed.
			# If ret[1] == 0 then command queue was full.
			while ret[0] == 0 or ret[1] == 0:
				ret = self._driver.Steps(base, rear, fore, baseDir, rearDir, foreDir)

	def _moveToAnglesSlice(self, baseAngle, rearArmAngle, foreArmAngle, \
									carryOverStepsBase, carryOverStepsRear, carryOverStepsFore):

		baseStepLocation = baseAngle * baseActualStepsPerRevolution / piTwo
		rearArmStepLocation = abs(rearArmAngle * rearArmActualStepsPerRevolution / piTwo)
		foreArmStepLocation = abs(foreArmAngle * foreArmActualStepsPerRevolution / piTwo)

		self._debug("Base Step Location", baseStepLocation)
		self._debug("Rear Arm Step Location", rearArmStepLocation)
		self._debug("Forearm Step Location", foreArmStepLocation)

		self._debug('self._baseSteps', self._baseSteps)
		self._debug('self._rearSteps', self._rearSteps)
		self._debug('self._foreSteps', self._foreSteps)

		self._debug('carryOverStepsBase', carryOverStepsBase)
		self._debug('carryOverStepsRear', carryOverStepsRear)
		self._debug('carryOverStepsFore', carryOverStepsFore)
		baseDiff = baseStepLocation - self._baseSteps + carryOverStepsBase
		rearDiff = rearArmStepLocation - self._rearSteps + carryOverStepsRear
		foreDiff = foreArmStepLocation - self._foreSteps + carryOverStepsFore
		self._debug('baseDiff', baseDiff)
		self._debug('rearDiff', rearDiff)
		self._debug('foreDiff', foreDiff)

		# self._baseSteps = baseStepLocation
		# self._rearSteps = rearArmStepLocation
		# self._foreSteps = foreArmStepLocation

		baseSign = 1
		rearSign = 1
		foreSign = -1
		baseDir = 1
		rearDir = 1
		foreDir = 1

		if (baseDiff < 1):
			baseDir = 0
			baseSign = -1
		if (rearDiff < 1):
			rearDir = 0
			rearSign = -1
		if (foreDiff > 1):
			foreDir = 0
			foreSign = 1

		baseDiffAbs = abs(baseDiff)
		rearDiffAbs = abs(rearDiff)
		foreDiffAbs = abs(foreDiff)

		cmdBaseVal, actualStepsBase, leftStepsBase = self._driver.stepsToCmdValFloat(baseDiffAbs)
		cmdRearVal, actualStepsRear, leftStepsRear = self._driver.stepsToCmdValFloat(rearDiffAbs)
		cmdForeVal, actualStepsFore, leftStepsFore = self._driver.stepsToCmdValFloat(foreDiffAbs)

		if not self._fake:
			# Repeat until the command is buffered. May not be buffered if buffer is full.
			ret = (0, 0)
			while not ret[1]:
				ret = self._driver.Steps(cmdBaseVal, cmdRearVal, cmdForeVal, baseDir, rearDir, foreDir)

		return (actualStepsBase * baseSign, actualStepsRear * rearSign, actualStepsFore * foreSign,\
					leftStepsBase * baseSign, leftStepsRear * rearSign, leftStepsFore * foreSign)

	def _sliceStepsToValues(self, steps, duration):
		'''
		@param steps - number of steps to make
		@param duration - duration that it should take to execute all steps, in seconds
		'''
		ret = []
		num = long(duration / 0.02)
		chunk = (steps / num)
		for _ in range(num):
			ret.append(self._driver.stepsToCmdVal(chunk))
		return ret

	def freqToCmdVal(self, freq):
		'''
		See DobotDriver.freqToCmdVal()
		'''
		return self._driver.freqToCmdVal(freq)

	def moveWithSpeed(self, x, y, z, maxSpeed):
		speed = float(maxSpeed)
		xx = float(x)
		yy = float(y)
		zz = float(z)

		currBaseAngle = piTwo * self._baseSteps / baseActualStepsPerRevolution
		currRearAngle = piHalf - piTwo * self._rearSteps / rearArmActualStepsPerRevolution
		currForeAngle = piTwo * self._foreSteps / foreArmActualStepsPerRevolution
		currX, currY, currZ = self._ik.getCoordinatesFromAngles(currBaseAngle, currRearAngle, currForeAngle)

		vectX = xx - currX
		vectY = yy - currY
		vectZ = zz - currZ
		self._debug('moving from', currX, currY, currZ)
		self._debug('moving to', xx, yy, zz)
		self._debug('moving by', vectX, vectY, vectZ)

		distance = math.sqrt(math.pow(vectX, 2) + math.pow(vectY, 2) + math.pow(vectZ, 2))
		self._debug('distance to travel', distance)

		slices = distance / maxSpeed / 0.02
		self._debug('slices to do', slices)

		sliceX = vectX / slices
		sliceY = vectY / slices
		sliceZ = vectZ / slices
		self._debug('slicing by', sliceX, sliceY, sliceZ)

		commands = 1
		leftStepsBase = 0.0
		leftStepsRear = 0.0
		leftStepsFore = 0.0
		toPlot1 = []
		toPlot2 = []
		toPlot3 = []
		toPlot4 = []
		toPlot5 = []
		toPlot6 = []
		toPlot7 = []
		toPlot8 = []
		toPlot9 = []
		while commands < slices:
		 # or abs(leftStepsBase) > 0 or abs(leftStepsRear) > 0 or abs(leftStepsFore) > 0:
			nextX = currX + sliceX * commands
			nextY = currY + sliceY * commands
			nextZ = currZ + sliceZ * commands
			self._debug('==============================')
			self._debug('moving to', nextX, nextY, nextZ)
			# moveToAngles = self._ik.convert_cartesian_coordinate_to_arm_angles(nextX, nextY, nextZ)
			# # check that inverse kinematics did not run into a range error.
			# # If it does, it should return -999 for all angles, so check that.
			# if moveToAngles[0] == -999:
			# 	print 'Unreachable'
			# 	return

			# self._debug('ik base angle', moveToAngles[0])
			# self._debug('ik rear angle', moveToAngles[1])
			# self._debug('ik fore angle', moveToAngles[2])

			# moveToRearArmAngleFloat = moveToAngles[1]
			# moveToForeArmAngleFloat = moveToAngles[2]

			# transformedRearArmAngle = piHalf - moveToRearArmAngleFloat
			# # -90 different from c++ code, accounts for fact that arm starts at the c++ simulation's 90
			# # note that this line is different from the similar line in the move angles function.
			# # Has to do with the inverse kinematics function and the fact that the forearm angle is
			# # calculated relative to the rear arm angle.
			# transformedForeArmAngle = piThreeFourths + transformedRearArmAngle - moveToForeArmAngleFloat
			# # self._debug('transformed rear angle', transformedRearArmAngle)
			# # self._debug('transformed fore angle', transformedForeArmAngle)

			# # check that the final angles are mechanically valid. note that this check only considers final
			# # angles, and not angles while the arm is moving
			# # need to pass in real world angles
			# # real world base and rear arm angles are those returned by the ik function.
			# # real world fore arm angle is -1 * transformedForeArmAngle
			# if not self._ik.check_for_angle_limits_is_valid(moveToAngles[0], moveToAngles[1], -1 * transformedForeArmAngle):
			# 	return

			# movedStepsBase, movedStepsRear, movedStepsFore, leftStepsBase, leftStepsRear, leftStepsFore = \
			# 	self._moveToAnglesSlice(moveToAngles[0], transformedRearArmAngle, transformedForeArmAngle, \
			# 							leftStepsBase, leftStepsRear, leftStepsFore)


			'''
			http://www.learnaboutrobots.com/inverseKinematics.htm
			'''
			radiansToDegrees = 180.0 / math.pi

			lengthRearArm = 135.0
			lengthForearm = 160.0
			# Distance from joint3 to the center of the tool mounted on the end effector.
			distanceTool = 50.9
			lengthRearSquared = math.pow(lengthRearArm, 2)
			lengthForeSquared = math.pow(lengthForearm, 2)

			# Radius to the center of the tool.
			radiusTool = math.sqrt(math.pow(nextX, 2) + math.pow(nextY, 2))
			self._debug('radiusTool', radiusTool)
			# Radius to joint3.
			radius = radiusTool - distanceTool
			self._debug('radius', radius)
			baseAngle = math.atan2(nextY, nextX)
			self._debug('ik base angle', baseAngle)
			# X coordinate of joint3.
			jointX = radius * math.cos(baseAngle)
			self._debug('jointX', jointX)
			# Y coordinate of joint3.
			jointY = radius * math.sin(baseAngle)
			self._debug('jointY', jointY)
			actualZ = nextZ - (80.0 + 23.0)
			self._debug('actualZ', actualZ)
			# Imaginary segment connecting joint1 with joint2, squared.
			hypotenuseSquared = math.pow(actualZ, 2) + math.pow(radius, 2)
			hypotenuse = math.sqrt(hypotenuseSquared)
			self._debug('hypotenuse', hypotenuse)
			self._debug('hypotenuseSquared', hypotenuseSquared)

			q1 = math.atan2(actualZ, radius)
			self._debug('q1', q1)
			q2 = math.acos((lengthRearSquared - lengthForeSquared + hypotenuseSquared) / (2.0 * lengthRearArm * hypotenuse))
			self._debug('q2', q2)
			rearAngle = piHalf - (q1 + q2)
			self._debug('ik rear angle', rearAngle)
			foreAngle = piHalf - (math.acos((lengthRearSquared + lengthForeSquared - hypotenuseSquared) / (2.0 * lengthRearArm * lengthForearm)) - rearAngle)
			self._debug('ik fore angle', foreAngle)


################
			movedStepsBase, movedStepsRear, movedStepsFore, leftStepsBase, leftStepsRear, leftStepsFore = \
				self._moveToAnglesSlice(baseAngle, rearAngle, foreAngle, \
										leftStepsBase, leftStepsRear, leftStepsFore)

			self._debug('moved', movedStepsBase, movedStepsRear, movedStepsFore, 'steps')
			self._debug('leftovers', leftStepsBase, leftStepsRear, leftStepsFore)

			commands += 1

			self._baseSteps += movedStepsBase
			self._rearSteps += movedStepsRear
			self._foreSteps += movedStepsFore

			# time.sleep(0.1)
			# print "--==STEPS==--"
			# print self._baseSteps, self._rearSteps, self._foreSteps
			# print self._driver.GetCounters()
			# print "--=========--"

			# toPlot1.append(self._baseSteps)
			# toPlot2.append(self._rearSteps)
			# toPlot3.append(self._foreSteps)
			currBaseAngle = piTwo * self._baseSteps / baseActualStepsPerRevolution
			currRearAngle = piHalf - piTwo * self._rearSteps / rearArmActualStepsPerRevolution
			currForeAngle = piTwo * self._foreSteps / foreArmActualStepsPerRevolution
			cX, cY, cZ = self._ik.getCoordinatesFromAngles(currBaseAngle, currRearAngle, currForeAngle)
			toPlot1.append(cX)
			toPlot2.append(cY)
			toPlot3.append(cZ)
			toPlot4.append(nextX)
			toPlot5.append(nextY)
			toPlot6.append(nextZ)
			toPlot7.append(cX - nextX)
			toPlot8.append(cY - nextY)
			toPlot9.append(cZ - nextZ)


		linewidth = 1.0
		# line, = plt.plot(toPlot, 'ro')
		# line.set_antialiased(False)
		plt.subplot(131)
		line, = plt.plot(toPlot4, linewidth=linewidth)
		line, = plt.plot(toPlot5, linewidth=linewidth)
		line, = plt.plot(toPlot6, linewidth=linewidth)
		plt.subplot(132)
		line, = plt.plot(toPlot1, linewidth=linewidth)
		line, = plt.plot(toPlot2, linewidth=linewidth)
		line, = plt.plot(toPlot3, linewidth=linewidth)
		plt.subplot(133)
		line, = plt.plot(toPlot7, linewidth=linewidth)
		line, = plt.plot(toPlot8, linewidth=linewidth)
		line, = plt.plot(toPlot9, linewidth=linewidth)
		# plt.ylabel('some numbers')
		# plt.show()
		# print "--==LAST STEPS READING==--"
		# time.sleep(5)
		# print self._baseSteps, self._rearSteps, self._foreSteps
		# print self._driver.GetCounters()
		# print "--=========--"


	def moveTo(self, x, y, z, duration):
		'''
		Moves the arm to the location with provided coordinates. Makes sure it takes exactly
		the amount of time provided in "duration" argument to get end effector to the location.
		All axes arrive at the same time.
		SDK keeps track of the current end effector pose.
		The origin is at the arm's base (the rotating base plate - joint1).
		Refer to docs/images/arm-description.png and docs/images/reference-frame.png to find
		more about reference frame and arm names.
		Current limitations:
		- rear arm cannot go beyond initial 90 degrees (backwards from initial vertical position),
		  so plan carefully the coordinates
		- forearm cannot go above initial horizontal position
		Not yet implemented:
		- the move is not linear as motors run the whole path at constant speed
		- acceleration/deceleration
		- rear arm cannot go beyond 90 degrees (see current limitations) and there are no checks
		  for that - it will simply go opposite direction instead
		'''
		xx = float(x)
		yy = float(y)
		zz = float(z)

		radiansToDegrees = 180.0 / math.pi

		b = 135.0
		c = 160.0
		b2 = math.pow(b, 2)
		c2 = math.pow(c, 2)

		self._debug('============================')

		r = math.sqrt(math.pow(xx, 2) + math.pow(yy, 2))
		self._debug('r', r)
		z2 = zz - (80.0 + 23.0)
		self._debug('z2', z2)
		# h = math.sqrt(math.pow(z2, 2) * math.pow(r, 2)) / 2.0
		# self._debug('h', h)
		d2 = math.pow(z2, 2) + math.pow(r, 2)
		d = math.sqrt(d2)
		self._debug('d', d)
		self._debug('d2', d2)

		q1 = math.atan2(z2, r)
		self._debug('q1', q1, q1 * radiansToDegrees)
		q2 = math.acos((b2 - c2 + d2) / (2.0 * b * d))
		self._debug('q2', q2, q2 * radiansToDegrees)
		rearAngle = piHalf - (q1 + q2)
		self._debug('ik rear angle', rearAngle, rearAngle * radiansToDegrees)
		foreAngle = piHalf - (math.acos((b2 + c2 - d2) / (2.0 * b * c)) - rearAngle)
		self._debug('ik fore angle', foreAngle, foreAngle * radiansToDegrees)
		baseAngle = math.atan2(yy, xx)
		self._debug('ik base angle', baseAngle, baseAngle * radiansToDegrees)

		self._moveArmToAngles(baseAngle, rearAngle, foreAngle, duration)


		# moveToAngles = self._ik.convert_cartesian_coordinate_to_arm_angles(xx, yy, zz)
		# # check that inverse kinematics did not run into a range error.
		# # If it does, it should return -999 for all angles, so check that.
		# if moveToAngles[0] == -999:
		# 	print 'Unreachable'
		# 	return

		# self._debug('ik base angle', moveToAngles[0])
		# self._debug('ik rear angle', moveToAngles[1])
		# self._debug('ik fore angle', moveToAngles[2])

		# moveToRearArmAngleFloat = moveToAngles[1]
		# moveToForeArmAngleFloat = moveToAngles[2]

		# transformedRearArmAngle = 90.0 - moveToRearArmAngleFloat
		# -90 different from c++ code, accounts for fact that arm starts at the c++ simulation's 90
		# note that this line is different from the similar line in the move angles function.
		# Has to do with the inverse kinematics function and the fact that the forearm angle is
		# calculated relative to the rear arm angle.
		# transformedForeArmAngle = 270.0 + transformedRearArmAngle - moveToForeArmAngleFloat
		# self._debug('transformed rear angle', transformedRearArmAngle)
		# self._debug('transformed fore angle', transformedForeArmAngle)

		# check that the final angles are mechanically valid. note that this check only considers final
		# angles, and not angles while the arm is moving
		# need to pass in real world angles
		# real world base and rear arm angles are those returned by the ik function.
		# real world fore arm angle is -1 * transformedForeArmAngle
		# if not self._ik.check_for_angle_limits_is_valid(moveToAngles[0], moveToAngles[1], -1 * transformedForeArmAngle):
		# 	return

		# self._moveArmToAngles(moveToAngles[0], transformedRearArmAngle, transformedForeArmAngle, duration)

	def CalibrateJoint(self, joint, forwardCommand, backwardCommand, direction, pin, pinMode, pullup):
		'''
		See DobotDriver.CalibrateJoint()
		'''
		return self._driver.CalibrateJoint(joint, forwardCommand, backwardCommand, direction, pin, pinMode, pullup)

	def EmergencyStop(self):
		'''
		See DobotDriver.EmergencyStop()
		'''

		return self._driver.EmergencyStop()

	def LaserOn(self, on):
		self._driver.LaserOn(on)


