import serial
import threading
import time
from serial import SerialException
from DobotDriver import DobotDriver
from DobotInverseKinematics import DobotInverseKinematics
import timeit

# The NEMA 17 stepper motors that Dobot uses are 200 steps per revolution.
stepperMotorStepsPerRevolution = 200.0
# FPGA board has all stepper drivers' stepping pins set to microstepping.
baseMicrosteppingMultiplier = 16.0
upperArmMicrosteppingMultiplier = 16.0
lowerArmMicrosteppingMultiplier = 16.0
# The NEMA 17 stepper motors Dobot uses are connected to a planetary gearbox, the black cylinders
# with 10:1 reduction ratio
stepperPlanetaryGearBoxMultiplier = 10.0

# calculate the actual number of steps it takes for each stepper motor to rotate 360 degrees
baseActualStepsPerRevolution = stepperMotorStepsPerRevolution * baseMicrosteppingMultiplier * stepperPlanetaryGearBoxMultiplier
upperArmActualStepsPerRevolution = stepperMotorStepsPerRevolution * upperArmMicrosteppingMultiplier * stepperPlanetaryGearBoxMultiplier
lowerArmActualStepsPerRevolution = stepperMotorStepsPerRevolution * lowerArmMicrosteppingMultiplier * stepperPlanetaryGearBoxMultiplier

class Dobot:
	def __init__(self, port, rate=115200, timeout=0.025):
		self._driver = DobotDriver(port, rate)
		self._driver.Open(timeout)
		self._ik = DobotInverseKinematics()
		self._x = 160.0
		self._y = 0.0
		self._z = 215.0
		self._baseAngle = 0.0
		self._rearAngle = 90.0
		self._frontAngle = 0.0
		self._baseSteps = long(0)
		self._rearSteps = long(0)
		self._frontSteps = long(0)

	def _reverseBits32(self, val):
		# return long(bin(val&0xFFFFFFFF)[:1:-1], 2)
		return int('{0:032b}'.format(val)[::-1], 2)

	def _freqToCmdVal(self, freq):
		'''
		Converts stepping frequency into a command value that dobot takes.
		'''
		if freq == 0:
			return 0x0242f000;
		return this._reverseBits32(long(25000000/freq))

	def _stepsToCmdVal(self, steps):
		'''
		Converts number of steps for dobot to do in 20ms into a command value that dobot
		takes to set the stepping frequency.
		'''
		if steps == 0:
			return 0x0242f000;
		return self._reverseBits32(long(500000/steps))

	def moveTo(self, x, y, z):
		xx = float(x)
		yy = float(y)
		zz = float(z)
		moveToAngles = self._ik.convert_cartesian_coordinate_to_arm_angles(xx, yy, zz)
		# check that inverse kinematics did not run into a range error.
		# If it does, it should return -999 for all angles, so check that.
		if moveToAngles[0] == -999:
			print 'Unreachable'
			return

		print 'ik base angle', moveToAngles[0]
		print 'ik upper angle', moveToAngles[1]
		print 'ik lower angle', moveToAngles[2]

		moveToUpperArmAngleFloat = moveToAngles[1]
		moveToLowerArmAngleFloat = moveToAngles[2]

		transformedUpperArmAngle = 90.0 - moveToUpperArmAngleFloat
		# -90 different from c++ code, accounts for fact that arm starts at the c++ simulation's 90
		# note that this line is different from the similar line in the move angles function.
		# Has to do with the inverse kinematics function and the fact that the lower arm angle is
		# calculated relative to the upper arm angle.
		transformedLowerArmAngle = 270.0 + transformedUpperArmAngle - moveToLowerArmAngleFloat
		print 'transformed upper angle', transformedUpperArmAngle
		print 'transformed lower angle', transformedLowerArmAngle

		# check that the final angles are mechanically valid. note that this check only considers final
		# angles, and not angles while the arm is moving
		# need to pass in real world angles
		# real world base and upper arm angles are those returned by the ik function.
		# real world lower arm angle is -1 * transformedLowerArmAngle
		if not self._ik.check_for_angle_limits_is_valid(moveToAngles[0], moveToAngles[1], -1 * transformedLowerArmAngle):
			return

		self._moveArmToAngles(moveToAngles[0], transformedUpperArmAngle, transformedLowerArmAngle)

	def _moveArmToAngles(self, baseAngle, upperArmAngle, lowerArmAngle):
		self._baseAngle = baseAngle
		self._rearAngle = upperArmAngle
		self._frontAngle = lowerArmAngle

		baseStepNumber = long((abs(baseAngle) / 360.0) * baseActualStepsPerRevolution + 0.5)
		upperArmStepNumber = long((abs(upperArmAngle) / 360.0) * upperArmActualStepsPerRevolution + 0.5)
		lowerArmStepNumber = long((abs(lowerArmAngle) / 360.0) * lowerArmActualStepsPerRevolution + 0.5)

		print "Base Step Number", baseStepNumber
		print "Upper Arm Step Number", upperArmStepNumber
		print "Lower Arm Step Number", lowerArmStepNumber

		baseDiff = baseStepNumber - self._baseSteps
		rearDiff = upperArmStepNumber - self._rearSteps
		print 'rearDiff', rearDiff
		frontDiff = lowerArmStepNumber - self._frontSteps

		self._baseSteps = baseStepNumber
		self._rearSteps = upperArmStepNumber
		self._frontSteps = lowerArmStepNumber

		baseDir = 1
		rearDir = 1
		frontDir = 1

		if (baseDiff < 1):
			baseDir = 0
		if (rearDiff < 1):
			rearDir = 0
		if (frontDiff > 1):
			frontDir = 0

		baseSliced = self._sliceStepsToValues(abs(baseDiff), 1)
		rearSliced = self._sliceStepsToValues(abs(rearDiff), 1)
		frontSliced = self._sliceStepsToValues(abs(frontDiff), 1)

		for base, upper, lower in zip(baseSliced, rearSliced, frontSliced):
			ret = [0, 0]
			# If ret[0] == 0 then command timed out or crc failed.
			# If ret[1] == 0 then command queue was full.
			while ret[0] == 0 or ret[1] == 0:
				ret = self._driver.Steps(base, upper, lower, baseDir, rearDir, frontDir)

	def _sliceStepsToValues(self, steps, duration):
		'''
		@param steps - number of steps to make
		@param duration - duration that it should take to execute all steps, in seconds
		'''
		ret = []
		num = long(duration / 0.02)
		chunk = (steps / num)
		for _ in range(num):
			ret.append(self._stepsToCmdVal(chunk))
		return ret



if __name__ == '__main__':

	dobot = Dobot('/dev/tty.usbmodem1421')

	# while True:
	dobot.moveTo(160.0, 0.0, 150.0)
	print '======================================'
	dobot.moveTo(160.0, 60.0, 150.0)
	print '======================================'
	dobot.moveTo(220.0, 60.0, 150.0)
	print '======================================'
	dobot.moveTo(220.0, 0.0, 150.0)
	print '======================================'
	dobot.moveTo(160.0, 0.0, 150.0)
	print '======================================'
	dobot.moveTo(160.0, 60.0, 150.0)
	print '======================================'
	dobot.moveTo(220.0, 60.0, 150.0)
	print '======================================'
	dobot.moveTo(220.0, 0.0, 150.0)
	print '======================================'
	dobot.moveTo(160.0, 0.0, 150.0)
	print '======================================'
	dobot.moveTo(160.0, 0.0, 215.0)
