"""
open-dobot inverse kinematics.

Implements inverse and forward kinematics functions.

Find firmware, driver and SDK at https://github.com/maxosprojects/open-dobot

Author: maxosprojects (March 18 2016)
Additional Authors: <put your name here>

Version 1.2.2

License: MIT
"""
import math
import sys

# Dimentions in mm
lengthRearArm = 135.0
lengthFrontArm = 160.0
# Horizontal distance from Joint3 to the center of the tool mounted on the end effector.
distanceTool = 50.9
# Joint1 height.
heightFromGround = 80.0 + 23.0

lengthRearSquared = pow(lengthRearArm, 2)
lengthFrontSquared = pow(lengthFrontArm, 2)

armSquaredConst = pow(lengthRearArm, 2) + pow(lengthFrontArm, 2)
armDoubledConst = 2.0 * lengthRearArm * lengthFrontArm
radiansToDegrees = 180.0 / math.pi
degreesToRadians = math.pi / 180.0

piHalf = math.pi / 2.0
piTwo = math.pi * 2.0
piThreeFourths = math.pi * 3.0 / 4.0

class DobotKinematics:
	def __init__(self, debug=False):
		self._debugOn = debug

	def _debug(self, *args):
		if self._debugOn:
			# Since "print" is not a function the expansion (*) cannot be used
			# as it is not an operator. So this is a workaround.
			for arg in args:
				sys.stdout.write(str(arg))
				sys.stdout.write(' ')
			print('')

	def coordinatesFromAngles(self, baseAngle, rearArmAngle, frontArmAngle):
		radius = lengthRearArm * math.cos(rearArmAngle) + lengthFrontArm * math.cos(frontArmAngle) + distanceTool
		x = radius * math.cos(baseAngle)
		y = radius * math.sin(baseAngle)
		z = heightFromGround - lengthFrontArm * math.sin(frontArmAngle) + lengthRearArm * math.sin(rearArmAngle)

		return (x, y, z)

	def anglesFromCoordinates(self, x, y, z):
		'''
		http://www.learnaboutrobots.com/inverseKinematics.htm
		'''
		# Radius to the center of the tool.
		radiusTool = math.sqrt(pow(x, 2) + pow(y, 2))
		self._debug('radiusTool', radiusTool)
		# Radius to joint3.
		radius = radiusTool - distanceTool
		self._debug('radius', radius)
		baseAngle = math.atan2(y, x)
		self._debug('ik base angle', baseAngle)
		# X coordinate of joint3.
		jointX = radius * math.cos(baseAngle)
		self._debug('jointX', jointX)
		# Y coordinate of joint3.
		jointY = radius * math.sin(baseAngle)
		self._debug('jointY', jointY)
		actualZ = z - heightFromGround
		self._debug('actualZ', actualZ)
		# Imaginary segment connecting joint1 with joint2, squared.
		hypotenuseSquared = pow(actualZ, 2) + pow(radius, 2)
		hypotenuse = math.sqrt(hypotenuseSquared)
		self._debug('hypotenuse', hypotenuse)
		self._debug('hypotenuseSquared', hypotenuseSquared)

		q1 = math.atan2(actualZ, radius)
		self._debug('q1', q1)
		q2 = math.acos((lengthRearSquared - lengthFrontSquared + hypotenuseSquared) / (2.0 * lengthRearArm * hypotenuse))
		self._debug('q2', q2)
		rearAngle = piHalf - (q1 + q2)
		self._debug('ik rear angle', rearAngle)
		frontAngle = piHalf - (math.acos((lengthRearSquared + lengthFrontSquared - hypotenuseSquared) / (2.0 * lengthRearArm * lengthFrontArm)) - rearAngle)
		self._debug('ik front angle', frontAngle)

		return (baseAngle, rearAngle, frontAngle)

	def get_distance_from_origin_to_cartesian_point_3D(self, x, y, z):
		#get distance from origin (0,0,0) to end point in 3D using pythagorean thm in 3D; distance = sqrt(x^2+y^2+z^2)
		distanceToEndPoint = math.sqrt( pow(x,2) + pow(y,2) + pow(z,2) )
		return distanceToEndPoint

	# angles passed as arguments here should be real world angles (horizontal = 0, below is negative, above is positive)
	# i.e. they should be set up the same way as the unit circle is
	def check_for_angle_limits_is_valid(self, baseAngle, rearArmAngle, foreArmAngle):
		ret = True
		# implementing limit switches and IMUs will make this function more accurate and allow the user to calibrate the limits
		# necessary for this function.
		# Not currently checking the base angle

		# check the rearArmAngle
		# max empirically determined to be around 107 - 108 degrees. Using 105.
		# min empirically determined to be around -23/24 degrees. Using -20.
		if (-20 > rearArmAngle > 105):
			print('Rear arm angle out of range')
			ret = False

		# check the foreArmAngle
		# the valid forearm angle is dependent on the rear arm angle. The real world angle of the forearm
		# (0 degrees = horizontal) needs to be evaluated.
		# min empirically determined to be around -105 degrees. Using -102.
		# max empirically determined to be around 21 degrees. Using 18.
		if (-102 > foreArmAngle > 18):
			print('Fore arm angle out of range')
			ret = False

		return ret

