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

Version: 0.4.0

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
	def __init__(self, port, rate=115200, timeout=0.025, debug=False):
		self._debugOn = debug
		self._driver = DobotDriver(port, rate)
		self._driver.Open(timeout)
		self._ik = DobotInverseKinematics(debug=debug)
		# Initialize arms current configuration from accelerometers
		accels = self._driver.GetAccelerometers()
		accelRear = accels[1]
		accelFore = accels[2]
		rearAngle = math.pi / 2 - self._driver.accelToRadians(accelRear, accelOffsets[0])
		foreAngle = self._driver.accelToRadians(accelFore, accelOffsets[1])
		self._baseSteps = long(0)
		self._rearSteps = long((rearAngle / math.pi / 2.0) * rearArmActualStepsPerRevolution + 0.5)
		self._foreSteps = long((foreAngle / math.pi / 2.0) * foreArmActualStepsPerRevolution + 0.5)

	def _debug(self, *args):
		if self._debugOn:
			# Since "print" is not a function the expansion (*) cannot be used
			# as it is not an operator. So this is a workaround.
			for arg in args:
				print arg,
			print

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
		moveToAngles = self._ik.convert_cartesian_coordinate_to_arm_angles(xx, yy, zz)
		# check that inverse kinematics did not run into a range error.
		# If it does, it should return -999 for all angles, so check that.
		if moveToAngles[0] == -999:
			print 'Unreachable'
			return

		self._debug('ik base angle', moveToAngles[0])
		self._debug('ik rear angle', moveToAngles[1])
		self._debug('ik fore angle', moveToAngles[2])

		moveToRearArmAngleFloat = moveToAngles[1]
		moveToForeArmAngleFloat = moveToAngles[2]

		transformedRearArmAngle = 90.0 - moveToRearArmAngleFloat
		# -90 different from c++ code, accounts for fact that arm starts at the c++ simulation's 90
		# note that this line is different from the similar line in the move angles function.
		# Has to do with the inverse kinematics function and the fact that the forearm angle is
		# calculated relative to the rear arm angle.
		transformedForeArmAngle = 270.0 + transformedRearArmAngle - moveToForeArmAngleFloat
		self._debug('transformed rear angle', transformedRearArmAngle)
		self._debug('transformed fore angle', transformedForeArmAngle)

		# check that the final angles are mechanically valid. note that this check only considers final
		# angles, and not angles while the arm is moving
		# need to pass in real world angles
		# real world base and rear arm angles are those returned by the ik function.
		# real world fore arm angle is -1 * transformedForeArmAngle
		if not self._ik.check_for_angle_limits_is_valid(moveToAngles[0], moveToAngles[1], -1 * transformedForeArmAngle):
			return

		self._moveArmToAngles(moveToAngles[0], transformedRearArmAngle, transformedForeArmAngle, duration)

	def _moveArmToAngles(self, baseAngle, rearArmAngle, foreArmAngle, duration):
		self._baseAngle = baseAngle
		self._rearAngle = rearArmAngle
		self._foreAngle = foreArmAngle
		dur = float(duration)

		baseStepLocation = long((baseAngle / 360.0) * baseActualStepsPerRevolution + 0.5)
		rearArmStepLocation = long((abs(rearArmAngle) / 360.0) * rearArmActualStepsPerRevolution + 0.5)
		foreArmStepLocation = long((abs(foreArmAngle) / 360.0) * foreArmActualStepsPerRevolution + 0.5)

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


