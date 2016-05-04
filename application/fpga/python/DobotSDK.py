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
import sys
# import matplotlib.pyplot as plt

# Workaround to support Python 2/3
if sys.version_info > (3,):
	long = int

piHalf = math.pi / 2.0
piTwo = math.pi * 2.0
piThreeFourths = math.pi * 3.0 / 4.0
# calibration-tool.py for details
accelOffsets = (1024, 1024)

radiansToDegrees = 180.0 / math.pi

lengthRearArm = 135.0
lengthForearm = 160.0
# Distance from joint3 to the center of the tool mounted on the end effector.
distanceTool = 50.9
heightFromBase = 80.0 + 23.0
lengthRearSquared = pow(lengthRearArm, 2)
lengthForeSquared = pow(lengthForearm, 2)

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
		self._toolRotation = 0
		self._gripper = 480
		# Initialize arms current configuration from accelerometers
		if fake:
			self._baseSteps = long(0)
			self._rearSteps = long(0)
			self._foreSteps = long(0)
		else:
			self._initializeAccelerometers()

	def _debug(self, *args):
		if self._debugOn:
			# Since "print" is not a function the expansion (*) cannot be used
			# as it is not an operator. So this is a workaround.
			for arg in args:
				sys.stdout.write(str(arg))
				sys.stdout.write(' ')
			print('')

	def _initializeAccelerometers(self):
		print("--=========--")
		accels = self._driver.GetAccelerometers()
		accelRear = accels[1]
		accelFore = accels[2]
		rearAngle = math.pi / 2 - self._driver.accelToRadians(accelRear, accelOffsets[0])
		foreAngle = self._driver.accelToRadians(accelFore, accelOffsets[1])
		self._baseSteps = long(0)
		self._rearSteps = long((rearAngle / math.pi / 2.0) * rearArmActualStepsPerRevolution + 0.5)
		self._foreSteps = long((foreAngle / math.pi / 2.0) * foreArmActualStepsPerRevolution + 0.5)
		self._driver.SetCounters(self._baseSteps, self._rearSteps, self._foreSteps)
		print("Initializing with steps:", self._baseSteps, self._rearSteps, self._foreSteps)
		print("Reading back what was set:", self._driver.GetCounters())
		currBaseAngle = piTwo * self._baseSteps / baseActualStepsPerRevolution
		currRearAngle = piHalf - piTwo * self._rearSteps / rearArmActualStepsPerRevolution
		currForeAngle = piTwo * self._foreSteps / foreArmActualStepsPerRevolution
		print('Current estimated coordinates:', self._getCoordinatesFromAngles(currBaseAngle, currRearAngle, currForeAngle))
		print("--=========--")

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
									carryOverStepsBase, carryOverStepsRear, carryOverStepsFore, \
									toolRotation):

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
				ret = self._driver.Steps(cmdBaseVal, cmdRearVal, cmdForeVal, baseDir, rearDir, foreDir, self._gripper, int(toolRotation))

		return (actualStepsBase * baseSign, actualStepsRear * rearSign, actualStepsFore * foreSign,\
					leftStepsBase * baseSign, leftStepsRear * rearSign, leftStepsFore * foreSign)

	def freqToCmdVal(self, freq):
		'''
		See DobotDriver.freqToCmdVal()
		'''
		return self._driver.freqToCmdVal(freq)

	def _moveTo(self, x, y, z, duration):
		'''
		TEMPORARILY UNAVALAILABLE
		Left for future refactor.

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
		b2 = pow(b, 2)
		c2 = pow(c, 2)

		self._debug('============================')

		r = math.sqrt(pow(xx, 2) + pow(yy, 2))
		self._debug('r', r)
		z2 = zz - (80.0 + 23.0)
		self._debug('z2', z2)
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

	def _getCoordinatesFromAngles(self, baseAngle, rearArmAngle, foreArmAngle):
		radius = lengthRearArm * math.cos(rearArmAngle) + lengthForearm * math.cos(foreArmAngle) + distanceTool
		x = radius * math.cos(baseAngle)
		y = radius * math.sin(baseAngle)
		z = lengthRearArm * math.sin(rearArmAngle) + heightFromBase - lengthForearm * math.sin(foreArmAngle)

		return (x, y, z)

	def MoveWithSpeed(self, x, y, z, maxSpeed, accel=None, toolRotation=None):
		'''
		For toolRotation see DobotDriver.Steps() function description (servoRot parameter).
		'''
		maxVel = float(maxSpeed)
		xx = float(x)
		yy = float(y)
		zz = float(z)
		
		if toolRotation == None:
			toolRotation = self._toolRotation
		elif toolRotation > 1024:
			toolRotation = 1024
		elif toolRotation < 0:
			toolRotation = 0

		accelf = None
		# Set 100% acceleration to equal maximum velocity if it wasn't provided
		if accel == None:
			accelf = maxVel
		else:
			accelf = float(accel)

		print("--=========--")
		self._debug('maxVel', maxVel)
		self._debug('accelf', accelf)

		currBaseAngle = piTwo * self._baseSteps / baseActualStepsPerRevolution
		currRearAngle = piHalf - piTwo * self._rearSteps / rearArmActualStepsPerRevolution
		currForeAngle = piTwo * self._foreSteps / foreArmActualStepsPerRevolution
		currX, currY, currZ = self._getCoordinatesFromAngles(currBaseAngle, currRearAngle, currForeAngle)

		vectX = xx - currX
		vectY = yy - currY
		vectZ = zz - currZ
		self._debug('moving from', currX, currY, currZ)
		self._debug('moving to', xx, yy, zz)
		self._debug('moving by', vectX, vectY, vectZ)

		distance = math.sqrt(pow(vectX, 2) + pow(vectY, 2) + pow(vectZ, 2))
		self._debug('distance to travel', distance)

		# If half the distance is reached before reaching maxSpeed with the given acceleration, then actual
		# maximum velocity will be lower, hence total number of slices is determined from half the distance
		# and acceleration.
		distToReachMaxSpeed = pow(maxVel, 2) / (2.0 * accelf)
		if distToReachMaxSpeed * 2.0 >= distance:
			timeToAccel = math.sqrt(distance / accelf)
			accelSlices = timeToAccel * 50.0
			timeFlat = 0
			flatSlices = 0
			maxVel = math.sqrt(distance * accelf)
		# Or else number of slices when velocity does not change is greater than zero.
		else:
			timeToAccel = maxVel / accelf
			accelSlices = timeToAccel * 50.0
			timeFlat = (distance - distToReachMaxSpeed * 2.0) / maxVel
			flatSlices = timeFlat * 50.0

		slices = accelSlices * 2.0 + flatSlices
		self._debug('slices to do', slices)
		self._debug('accelSlices', accelSlices)
		self._debug('flatSlices', flatSlices)

		# Acceleration/deceleration in respective axes
		accelX = (accelf * vectX) / distance
		accelY = (accelf * vectY) / distance
		accelZ = (accelf * vectZ) / distance
		self._debug('accelXYZ', accelX, accelY, accelZ)

		# Vectors in respective axes to complete acceleration/deceleration
		segmentAccelX = accelX * pow(timeToAccel, 2) / 2.0
		segmentAccelY = accelY * pow(timeToAccel, 2) / 2.0
		segmentAccelZ = accelZ * pow(timeToAccel, 2) / 2.0
		self._debug('segmentAccelXYZ', segmentAccelX, segmentAccelY, segmentAccelZ)

		# Maximum velocity in respective axes for the segment with constant velocity
		maxVelX = (maxVel * vectX) / distance
		maxVelY = (maxVel * vectY) / distance
		maxVelZ = (maxVel * vectZ) / distance
		self._debug('maxVelXYZ', maxVelX, maxVelY, maxVelZ)

		# Vectors in respective axes for the segment with constant velocity
		segmentFlatX = maxVelX * timeFlat
		segmentFlatY = maxVelY * timeFlat
		segmentFlatZ = maxVelZ * timeFlat
		self._debug('segmentFlatXYZ', segmentFlatX, segmentFlatY, segmentFlatZ)

		segmentToolRotation = (toolRotation - self._toolRotation) / slices
		self._debug('segmentToolRotation', segmentToolRotation)

		# sliceX = vectX / slices
		# sliceY = vectY / slices
		# sliceZ = vectZ / slices
		# self._debug('slicing by', sliceX, sliceY, sliceZ)

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
			self._debug('==============================')
			self._debug('slice #', commands)
			# If accelerating
			if commands <= accelSlices:
				t2half = pow(commands / 50.0, 2) / 2.0
				nextX = currX + accelX * t2half
				nextY = currY + accelY * t2half
				nextZ = currZ + accelZ * t2half
			# If decelerating
			elif commands >= accelSlices + flatSlices:
				t2half = pow((slices - commands) / 50.0, 2) / 2.0
				nextX = currX + segmentAccelX * 2.0 + segmentFlatX - accelX * t2half
				nextY = currY + segmentAccelY * 2.0 + segmentFlatY - accelY * t2half
				nextZ = currZ + segmentAccelZ * 2.0 + segmentFlatZ - accelZ * t2half
			# Or else moving at maxSpeed
			else:
				t = abs(commands - accelSlices) / 50.0
				nextX = currX + segmentAccelX + maxVelX * t
				nextY = currY + segmentAccelY + maxVelY * t
				nextZ = currZ + segmentAccelZ + maxVelZ * t
			self._debug('moving to', nextX, nextY, nextZ)

			nextToolRotation = self._toolRotation + (segmentToolRotation * commands)
			self._debug('nextToolRotation', nextToolRotation)

			'''
			http://www.learnaboutrobots.com/inverseKinematics.htm
			'''

			# Radius to the center of the tool.
			radiusTool = math.sqrt(pow(nextX, 2) + pow(nextY, 2))
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
			actualZ = nextZ - heightFromBase
			self._debug('actualZ', actualZ)
			# Imaginary segment connecting joint1 with joint2, squared.
			hypotenuseSquared = pow(actualZ, 2) + pow(radius, 2)
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

			movedStepsBase, movedStepsRear, movedStepsFore, leftStepsBase, leftStepsRear, leftStepsFore = \
				self._moveToAnglesSlice(baseAngle, rearAngle, foreAngle, \
										leftStepsBase, leftStepsRear, leftStepsFore, \
										nextToolRotation)

			self._debug('moved', movedStepsBase, movedStepsRear, movedStepsFore, 'steps')
			self._debug('leftovers', leftStepsBase, leftStepsRear, leftStepsFore)

			commands += 1

			self._baseSteps += movedStepsBase
			self._rearSteps += movedStepsRear
			self._foreSteps += movedStepsFore

			currBaseAngle = piTwo * self._baseSteps / baseActualStepsPerRevolution
			currRearAngle = piHalf - piTwo * self._rearSteps / rearArmActualStepsPerRevolution
			currForeAngle = piTwo * self._foreSteps / foreArmActualStepsPerRevolution
			cX, cY, cZ = self._getCoordinatesFromAngles(currBaseAngle, currRearAngle, currForeAngle)
			toPlot1.append(cX)
			toPlot2.append(cY)
			toPlot3.append(cZ)
			toPlot4.append(nextX)
			toPlot5.append(nextY)
			toPlot6.append(nextZ)
			toPlot7.append(cX - nextX)
			toPlot8.append(cY - nextY)
			toPlot9.append(cZ - nextZ)

		self._toolRotation = toolRotation

		# linewidth = 1.0
		# plt.subplot(131)
		# line, = plt.plot(toPlot4, linewidth=linewidth)
		# line, = plt.plot(toPlot5, linewidth=linewidth)
		# line, = plt.plot(toPlot6, linewidth=linewidth)
		# plt.subplot(132)
		# line, = plt.plot(toPlot1, linewidth=linewidth)
		# line, = plt.plot(toPlot2, linewidth=linewidth)
		# line, = plt.plot(toPlot3, linewidth=linewidth)
		# plt.subplot(133)
		# line, = plt.plot(toPlot7, linewidth=linewidth, label='x')
		# line, = plt.plot(toPlot8, linewidth=linewidth, label='y')
		# line, = plt.plot(toPlot9, linewidth=linewidth, label='z')
		# legend = plt.legend(loc='upper center', shadow=True)
		# plt.show()

	def Gripper(self, gripper):
		if gripper > 480:
			self._gripper = 480
		elif gripper < 208:
			self._gripper = 208
		else:
			self._gripper = gripper

		self._driver.Steps(0, 0, 0, 0, 0, 0, self._gripper, self._toolRotation)

	def Wait(self, waitTime):
		'''
		Makes the arm wait in current position for the specified period of time. The wait period is specified
		in seconds and can be fractions of seconds.
		The resolution of this command is up to 20ms.

		In order to make the arm wait a number of commands are issued to do nothing. Each command takes 20ms
		to execute by the arm.
		'''
		iterations = int(waitTime * 50)
		for i in range(iterations):
			ret = (0, 0)
			# Keep sending until buffered
			while not ret[0] or not ret[1]:
				ret = self._driver.Steps(0, 0, 0, 0, 0, 0, self._gripper, self._toolRotation)

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
		return self._driver.LaserOn(on)
	
	def PumpOn(self, on):
		return self._driver.PumpOn(on)
	
	def ValveOn(self, on):
		return self._driver.ValveOn(on)

