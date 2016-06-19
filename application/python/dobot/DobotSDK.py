"""
open-dobot SDK.

SDK providing high-level functions to control Dobot via the driver to open firmware, which, in turn, controls Dobot FPGA.
Abstracts specifics of commands sent to FPGA.
Find firmware and driver at https://github.com/maxosprojects/open-dobot

It is assumed that upon SDK initialization the arms are beetween 0 and 90 degrees - beetween their normal
horizontal and vertical positions.
Upon initialization accelerometers are read to figure out current arms' configuration. Accelerometers get confused
when rear arm leans backwards from the dobot base or when front arm bends towards the base.
Also, Inverse Kinematics at the moment don't account for when front arm is looking up (higher than it's
normal horizontal position). So be gentle and give dobot some feasible initial configuration in case it happened
to be beyond the mentioned limits.
Refer to docs/images/ to find more about reference frame, arm names and more.

SDK keeps track of the current end effector pose, thus in case the arm slips or motors are disabled while
in move (with the "Laser Adjustment" button) it has to be re-initialized and SDK re-initialized.

Author: maxosprojects (March 18 2016)
Additional Authors: <put your name here>

Version: 1.2.2

License: MIT
"""

import serial
import threading
import time
from serial import SerialException
from DobotDriver import DobotDriver
from DobotKinematics import *
import timeit
import math
import sys

# Workaround to support Python 2/3
if sys.version_info > (3,):
	long = int

# See calibrate-accelerometers.py for details
accelOffsets = (1024, 1024)

# Backlash in the motor reduction gears is actually 22 steps, but 5 is visually unnoticeable.
# It is a horrible thing to compensate a bad backlash in software, but the only other
# option is to physically rebuild Dobot to fix this problem.
backlash = 5

# The NEMA 17 stepper motors that Dobot uses are 200 steps per revolution.
stepperMotorStepsPerRevolution = 200.0
# FPGA board has all stepper drivers' stepping pins set to microstepping.
baseMicrosteppingMultiplier = 16.0
rearArmMicrosteppingMultiplier = 16.0
frontArmMicrosteppingMultiplier = 16.0
# The NEMA 17 stepper motors Dobot uses are connected to a planetary gearbox, the black cylinders
# with 10:1 reduction ratio
stepperPlanetaryGearBoxMultiplier = 10.0

# calculate the actual number of steps it takes for each stepper motor to rotate 360 degrees
baseActualStepsPerRevolution = stepperMotorStepsPerRevolution * baseMicrosteppingMultiplier * stepperPlanetaryGearBoxMultiplier
rearArmActualStepsPerRevolution = stepperMotorStepsPerRevolution * rearArmMicrosteppingMultiplier * stepperPlanetaryGearBoxMultiplier
frontArmActualStepsPerRevolution = stepperMotorStepsPerRevolution * frontArmMicrosteppingMultiplier * stepperPlanetaryGearBoxMultiplier

class Dobot:
	def __init__(self, port, rate=115200, timeout=0.025, debug=False, plot=False, fake=False):
		self._debugOn = debug
		self._fake = fake
		self._driver = DobotDriver(port, rate)
		if fake:
			self._driver._ramps = True
			self._driver._stepCoeff = 20000
			self._driver._stopSeq = 0
			self._driver._stepCoeffOver2 = self._driver._stepCoeff / 2
			self._driver._freqCoeff = self._driver._stepCoeff * 25
		else:
			self._driver.Open(timeout)
		self._plot = plot
		if plot:
			import matplotlib.pyplot as plt
			self._plt = plt
		self._kinematics = DobotKinematics(debug=debug)
		self._toolRotation = 0
		self._gripper = 480
		# Last directions to compensate backlash.
		self._lastBaseDirection = 0
		self._lastRearDirection = 0
		self._lastFrontDirection = 0
		# Initialize arms current configuration from accelerometers
		if fake:
			self._baseSteps = long(0)
			self._rearSteps = long(0)
			self._frontSteps = long(0)
		else:
			self.InitializeAccelerometers()

	def _debug(self, *args):
		if self._debugOn:
			# Since "print" is not a function the expansion (*) cannot be used
			# as it is not an operator. So this is a workaround.
			for arg in args:
				sys.stdout.write(str(arg))
				sys.stdout.write(' ')
			print('')

	def InitializeAccelerometers(self):
		print("--=========--")
		print("Initializing accelerometers")
		if self._driver.isFpga():
			# In FPGA v1.0 SPI accelerometers are read only when Arduino boots. The readings
			# are already available, so read once.
			ret = (0, 0, 0, 0, 0, 0, 0)
			while not ret[0]:
				ret = self._driver.GetAccelerometers()
			accelRearX = ret[1]
			accelFrontX = ret[4]
			rearAngle = piHalf - self._driver.accelToRadians(accelRearX, accelOffsets[0])
			frontAngle = self._driver.accelToRadians(accelFrontX, accelOffsets[1])
		else:
			# In RAMPS accelerometers are on I2C bus and can be read at any time. We need to
			# read them multiple times to get average as MPU-6050 have greater resolution but are noisy.
			# However, due to the interference from the way A4988 holds the motors if none of the
			# recommended measures to suppress interference are in place (see open-dobot wiki), or
			# in case accelerometers are not connected, we need to give up and assume some predefined pose.
			accelRearX = 0
			accelRearY = 0
			accelRearZ = 0
			accelFrontX = 0
			accelFrontY = 0
			accelFrontZ = 0
			successes = 0
			for i in range(20):
				ret = (0, 0, 0, 0, 0, 0, 0)
				attempts = 10
				while attempts:
					ret = self._driver.GetAccelerometers()
					if ret[0]:
						successes += 1
						accelRearX += ret[1]
						accelRearY += ret[2]
						accelRearZ += ret[3]
						accelFrontX += ret[4]
						accelFrontY += ret[5]
						accelFrontZ += ret[6]
						break
					attempts -= 1
			if successes > 0:
				divisor = float(successes)
				rearAngle = piHalf - self._driver.accel3DXToRadians(accelRearX / divisor, accelRearY / divisor, accelRearZ / divisor)
				frontAngle = -self._driver.accel3DXToRadians(accelFrontX / divisor, accelFrontY / divisor, accelFrontZ / divisor)
			else:
				print('Failed to read accelerometers. Make sure they are connected and interference is suppressed. See open-dobot wiki')
				print('Assuming rear arm vertical and front arm horizontal')
				rearAngle = 0
				frontAngle = -piHalf
		self._baseSteps = long(0)
		self._rearSteps = long((rearAngle / piTwo) * rearArmActualStepsPerRevolution + 0.5)
		self._frontSteps = long((frontAngle / piTwo) * frontArmActualStepsPerRevolution + 0.5)
		self._driver.SetCounters(self._baseSteps, self._rearSteps, self._frontSteps)
		print("Initializing with steps:", self._baseSteps, self._rearSteps, self._frontSteps)
		print("Reading back what was set:", self._driver.GetCounters())
		currBaseAngle = piTwo * self._baseSteps / baseActualStepsPerRevolution
		currRearAngle = piHalf - piTwo * self._rearSteps / rearArmActualStepsPerRevolution
		currFrontAngle = piTwo * self._frontSteps / frontArmActualStepsPerRevolution
		print('Current estimated coordinates:', self._kinematics.coordinatesFromAngles(currBaseAngle, currRearAngle, currFrontAngle))
		print("--=========--")

	def _moveArmToAngles(self, baseAngle, rearArmAngle, frontArmAngle, duration):
		self._baseAngle = baseAngle
		self._rearAngle = rearArmAngle
		self._frontAngle = frontArmAngle
		dur = float(duration)

		# baseStepLocation = long((baseAngle / 360.0) * baseActualStepsPerRevolution + 0.5)
		# rearArmStepLocation = long((abs(rearArmAngle) / 360.0) * rearArmActualStepsPerRevolution + 0.5)
		# frontArmStepLocation = long((abs(frontArmAngle) / 360.0) * frontArmActualStepsPerRevolution + 0.5)
		baseStepLocation = long(baseAngle * baseActualStepsPerRevolution / piTwo)
		rearArmStepLocation = long(rearArmAngle * rearArmActualStepsPerRevolution / piTwo)
		frontArmStepLocation = long(frontArmAngle * frontArmActualStepsPerRevolution / piTwo)

		self._debug("Base Step Location", baseStepLocation)
		self._debug("Rear Arm Step Location", rearArmStepLocation)
		self._debug("Frontarm Step Location", frontArmStepLocation)

		baseDiff = baseStepLocation - self._baseSteps
		rearDiff = rearArmStepLocation - self._rearSteps
		frontDiff = frontArmStepLocation - self._frontSteps
		self._debug('baseDiff', baseDiff)
		self._debug('rearDiff', rearDiff)
		self._debug('frontDiff', frontDiff)

		self._baseSteps = baseStepLocation
		self._rearSteps = rearArmStepLocation
		self._frontSteps = frontArmStepLocation

		baseDir = 1
		rearDir = 1
		frontDir = 1

		if (baseDiff < 1):
			baseDir = 0
		if (rearDiff < 1):
			rearDir = 0
		if (frontDiff > 1):
			frontDir = 0

		baseSliced = self._sliceStepsToValues(abs(baseDiff), dur)
		rearSliced = self._sliceStepsToValues(abs(rearDiff), dur)
		frontSliced = self._sliceStepsToValues(abs(frontDiff), dur)

		for base, rear, front in zip(baseSliced, rearSliced, frontSliced):
			ret = [0, 0]
			# If ret[0] == 0 then command timed out or crc failed.
			# If ret[1] == 0 then command queue was full.
			while ret[0] == 0 or ret[1] == 0:
				ret = self._driver.Steps(base, rear, front, baseDir, rearDir, frontDir)

	def _moveToAnglesSlice(self, baseAngle, rearArmAngle, frontArmAngle, toolRotation):

		baseStepLocation = baseAngle * baseActualStepsPerRevolution / piTwo
		rearArmStepLocation = abs(rearArmAngle * rearArmActualStepsPerRevolution / piTwo)
		frontArmStepLocation = abs(frontArmAngle * frontArmActualStepsPerRevolution / piTwo)

		self._debug("Base Step Location", baseStepLocation)
		self._debug("Rear Arm Step Location", rearArmStepLocation)
		self._debug("Front Arm Step Location", frontArmStepLocation)

		self._debug('self._baseSteps', self._baseSteps)
		self._debug('self._rearSteps', self._rearSteps)
		self._debug('self._frontSteps', self._frontSteps)

		baseDiff = baseStepLocation - self._baseSteps
		rearDiff = rearArmStepLocation - self._rearSteps
		frontDiff = frontArmStepLocation - self._frontSteps

		self._debug('baseDiff', baseDiff)
		self._debug('rearDiff', rearDiff)
		self._debug('frontDiff', frontDiff)

		baseSign = 1
		rearSign = 1
		frontSign = -1
		baseDir = 1
		rearDir = 1
		frontDir = 1

		if (baseDiff < 1):
			baseDir = 0
			baseSign = -1
		if (rearDiff < 1):
			rearDir = 0
			rearSign = -1
		if (frontDiff > 1):
			frontDir = 0
			frontSign = 1

		baseDiffAbs = abs(baseDiff)
		rearDiffAbs = abs(rearDiff)
		frontDiffAbs = abs(frontDiff)

		cmdBaseVal, actualStepsBase, leftStepsBase = self._driver.stepsToCmdValFloat(baseDiffAbs)
		cmdRearVal, actualStepsRear, leftStepsRear = self._driver.stepsToCmdValFloat(rearDiffAbs)
		cmdFrontVal, actualStepsFront, leftStepsFront = self._driver.stepsToCmdValFloat(frontDiffAbs)

		# Compensate for backlash.
		# For now compensate only backlash in the base motor as the backlash in the arm motors depends
		# on specific task (a laser/brush or push-pull tasks).
		if self._lastBaseDirection != baseDir and actualStepsBase > 0:
			cmdBaseVal, _ignore, _ignore = self._driver.stepsToCmdValFloat(baseDiffAbs + backlash)
			self._lastBaseDirection = baseDir
		# if self._lastRearDirection != rearDir and actualStepsRear > 0:
		# 	cmdRearVal, _ignore, _ignore = self._driver.stepsToCmdValFloat(rearDiffAbs + backlash)
		# 	self._lastRearDirection = rearDir
		# if self._lastFrontDirection != frontDir and actualStepsFront > 0:
		# 	cmdFrontVal, _ignore, _ignore = self._driver.stepsToCmdValFloat(frontDiffAbs + backlash)
		# 	self._lastFrontDirection = frontDir

		if not self._fake:
			# Repeat until the command is queued. May not be queued if queue is full.
			ret = (0, 0)
			while not ret[1]:
				ret = self._driver.Steps(cmdBaseVal, cmdRearVal, cmdFrontVal, baseDir, rearDir, frontDir, self._gripper, int(toolRotation))

		if self._plot:
			self._toPlot1.append(baseDiff)
			self._toPlot2.append(actualStepsBase * baseSign)

		return (actualStepsBase * baseSign, actualStepsRear * rearSign, actualStepsFront * frontSign,\
					leftStepsBase * baseSign, leftStepsRear * rearSign, leftStepsFront * frontSign)

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
		- front arm cannot go above initial horizontal position
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
		frontAngle = piHalf - (math.acos((b2 + c2 - d2) / (2.0 * b * c)) - rearAngle)
		self._debug('ik front angle', frontAngle, frontAngle * radiansToDegrees)
		baseAngle = math.atan2(yy, xx)
		self._debug('ik base angle', baseAngle, baseAngle * radiansToDegrees)

		self._moveArmToAngles(baseAngle, rearAngle, frontAngle, duration)

	def MoveWithSpeed(self, x, y, z, maxSpeed, accel=None, toolRotation=None):
		'''
		For toolRotation see DobotDriver.Steps() function description (servoRot parameter).
		'''

		if self._plot:
			self._toPlot1 = []
			self._toPlot2 = []

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
		currFrontAngle = piTwo * self._frontSteps / frontArmActualStepsPerRevolution
		currX, currY, currZ = self._kinematics.coordinatesFromAngles(currBaseAngle, currRearAngle, currFrontAngle)

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

		commands = 1
		leftStepsBase = 0.0
		leftStepsRear = 0.0
		leftStepsFront = 0.0
		if self._plot:
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

			(baseAngle, rearAngle, frontAngle) = self._kinematics.anglesFromCoordinates(nextX, nextY, nextZ)

			movedStepsBase, movedStepsRear, movedStepsFront, leftStepsBase, leftStepsRear, leftStepsFront = \
				self._moveToAnglesSlice(baseAngle, rearAngle, frontAngle, nextToolRotation)

			self._debug('moved', movedStepsBase, movedStepsRear, movedStepsFront, 'steps')
			self._debug('leftovers', leftStepsBase, leftStepsRear, leftStepsFront)

			commands += 1

			self._baseSteps += movedStepsBase
			self._rearSteps += movedStepsRear
			self._frontSteps += movedStepsFront

			currBaseAngle = piTwo * self._baseSteps / baseActualStepsPerRevolution
			currRearAngle = piHalf - piTwo * self._rearSteps / rearArmActualStepsPerRevolution
			currFrontAngle = piTwo * self._frontSteps / frontArmActualStepsPerRevolution
			cX, cY, cZ = self._kinematics.coordinatesFromAngles(currBaseAngle, currRearAngle, currFrontAngle)
			if self._plot:
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

		if self._plot:
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

			linewidth = 1.0
			self._plt.plot(self._toPlot1, linewidth=linewidth)
			self._plt.plot(self._toPlot2, linewidth=2.0)
			# make the y ticks integers, not floats
			yint = []
			locs, labels = self._plt.yticks()
			for each in locs:
			    yint.append(int(each))
			self._plt.yticks(yint)
			self._plt.show()

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
		See description in DobotDriver.Wait()
		'''
		self._driver.Wait(waitTime)

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

