"""
Implements inverse and forward kinematics functions, in addition to other movement calculation functions for the Dobot Arm
First Author: Mike Ferguson www.mikeahferguson.com 3/26/2016
Additional Authors (Add your name below): Max

License: MIT
"""
import math

"""
polar coordinates conversion: https://www.youtube.com/watch?v=L4v98ZZft68
IK 2D, 2DOF, revolute revolute: https://www.youtube.com/watch?v=cvzv3YxuoQE

algorithm

xy plane is paralell to surface dobot is on. z is perpendicular
1. first get distance, in xy plane, to current point from origin using forward kinematics, known angles, and pythagoreas thm. This is your radius. Your angle can be defined to be theta original. You now have your starting point in polar coordinates.
2. Ignore the desired z position data for now. Get the polar coordinates for the desired point. The radius is not important now. The angle is though. Subtracting the desired angle from the current angle gives you the number of degrees and direction to rotate the base.
3. The radius from step 1 (starting radius) gives you your current horizontal position (imagine it as x or y, doesn't matter). You also already know your current z position (potentially from step 1).
4. The radius from step 2 (desired radius) gives you your desired horizontal position (imagine it as x or y, doesn't matter). Of course, the user has already input the desired z position. This is now a 2D problem with two arms that each rotate (2 degrees of freedom)
5: use IK, see ik 2d video, to find number of degrees and direction to rotate upper and lower arms. Note that there are often two solutions. One (elbow down) is not possible.
6. Check that move is valid (e.g. not out of range, etc...)
7. move

default working angle units are radians
"""
#length and height dimensions in mm
heightFromBase = 80.0
lengthUpperArm = 135.0
lengthLowerArm = 160.0

armSquaredConst = pow(lengthUpperArm, 2) + pow(lengthLowerArm, 2)
armDoubledConst = 2.0 * lengthUpperArm * lengthLowerArm
radiansToDegrees = 180.0 / math.pi

'''
upperArm - rearArm
lowerArm - frontArm
'''

class DobotInverseKinematics:
    #input:
    #cartesian (x,y,z) coordinate
    #robot dimensions
    #output:
    #angles for robot arm base, upper, and lower arms in degree
    def convert_cartesian_coordinate_to_arm_angles(self, x, y, z):

        # do a general check to see if even a maximally stretched arm could reach the point
        # if it can't, return some dummy angle numbers of -999
        # note the base height correction in z
        distanceFromOriginToEndPoint = self.get_distance_from_origin_to_cartesian_point_3D(x, y, z - heightFromBase)
        print(str(distanceFromOriginToEndPoint))
        if (distanceFromOriginToEndPoint > (lengthUpperArm + lengthLowerArm)):
            return [-999,-999,-999]

        baseAngle = self.get_polar_coordinate_angle_from_cartesian_x_y_coordinate(x, y)
        radiusToEndPoint = self.get_polar_coordinate_radius_from_cartesian_x_y_coordinate(x, y)
        #note the correction for base height
        armAngles = self.get_arm_angles_from_radius_z_coordinate_using_2d_revolute_revolute_inverse_kinematics(radiusToEndPoint, z - heightFromBase)
        upperArmAngle = armAngles[0]
        lowerArmAngle = armAngles[1]

        #convert the angles to degrees when you return them
        return [baseAngle * radiansToDegrees, upperArmAngle * radiansToDegrees, lowerArmAngle * radiansToDegrees]

    def get_arm_angles_from_radius_z_coordinate_using_2d_revolute_revolute_inverse_kinematics(self, r, z):

        #eq is a dummy variable for the equation that it represents
        eq = (armSquaredConst - pow(r, 2) - pow(z, 2)) / armDoubledConst

        #the two angles are due to the two possible angles, elbow up or down. I always want elbow up.
        #Not sure which one is elbow up, guessing it's the positive sqrt equation for now. CHECK THIS!
        #note that pi radians = 180 degrees
        lowerArmAngle = math.pi - math.atan2(math.sqrt(1 - pow(eq, 2)), eq)
        #including the angle two
        lowerArmAngleAlternative = math.pi - math.atan2(-1 * math.sqrt(1 - pow(eq,2)), eq )
        lowerArmAngle = lowerArmAngleAlternative

        #note that the upperArmAngle is dependent on the lowerArmAngle. This makes sense.
        # can easily envision that upper arm would be at two different angles if arm is elbow up or down
        upperArmAngle = math.atan2(z,r) - math.atan2( (lengthLowerArm * math.sin(lowerArmAngle)) , (lengthUpperArm + (lengthLowerArm * math.cos(lowerArmAngle))) )

        return [upperArmAngle, lowerArmAngle]

    #input:
    #x,y coordinate
    #output
    #polar coordinate angle to x,y point in radians
    def get_polar_coordinate_angle_from_cartesian_x_y_coordinate(self, x, y):
        #use cartesian to polar conversion equations to get the angle
        #alternatively, this is just using the toa rule
        angle = math.atan2(y, x)
        return angle

    #input:
    #x,y coordinate
    #output
    #length of polar coordinate radius to x,y point
    def get_polar_coordinate_radius_from_cartesian_x_y_coordinate(self, x, y):
        #use cartesian to polar conversion equations to get the angle
        #alternatively, this is just using the circle equation rule
        radius = math.sqrt(pow(x,2) + pow(y,2))
        return radius

    def get_distance_from_origin_to_cartesian_point_3D(self, x, y, z):
        #get distance from origin (0,0,0) to end point in 3D using pythagorean thm in 3D; distance = sqrt(x^2+y^2+z^2)
        distanceToEndPoint = math.sqrt( pow(x,2) + pow(y,2) + pow(z,2) )
        return distanceToEndPoint

    def get_upper_arm_angle(self):
        #return the angle in degrees or radians for the upper arm from the accelerometer data and/or known theoretical angle
        return 45#or radians!

    def get_lower_arm_angle(self):
        #return the angle in degrees or radians for the lower arm from the accelerometer data and/or known theoretical angle
        return 45#or radians!

    def get_base_angle(self):
        #return the angle in degrees or radians for the base from the accelerometer data and/or known theoretical angle
        return 45#or radians!

    # angles passed as arguments here should be real world angles (horizontal = 0, below is negative, above is positive)
    # i.e. they should be set up the same way as the unit circle is
    def check_for_angle_limits_is_valid(self, baseAngle, upperArmAngle, lowerArmAngle):
        ret = True
        # implementing limit switches and IMUs will make this function more accurate and allow the user to calibrate the limits
        # necessary for this function.
        # Not currently checking the base angle

        # check the upperArmAngle
        # max empirically determined to be around 107 - 108 degrees. Using 105.
        # min empirically determined to be around -23/24 degrees. Using -20.
        if (-20 > upperArmAngle > 105):
            print 'Upper arm angle out of range'
            ret = False

        # check the lowerArmAngle
        # the valid Lower Arm angle is dependent on the upper arm angle. The real world angle of the lower arm (0 degrees = horizontal) needs to be evaluated.
        # min empirically determined to be around -105 degrees. Using -102.
        # max empirically determined to be around 21 degrees. Using 18.
        if (-102 > lowerArmAngle > 18):
            print 'Lower arm angle out of range'
            ret = False

        return ret

"""
    #get polar coordinates for the current point
    #radius is simply given by the base angle
    # can read the angles from the IMU and empirically determine the radius and angle - I'm using this approach since it should account for any build up in error, assuming accelerometers
    #are accurate!!!!
    #alternatively can just use pythagorean thm on theoretical current x,y data

    startUpperArmAngle = get_upper_arm_angle
    startLowerArmAngle = get_lower_arm_angle
    startBaseAngle = get_base_angle

    #could abstract this next bit into a 2D forward kinematics function and then just use the horizontal data returned
    #only care about the radius, so


    currentPosPolarCoordRadius = ???
    currentPosPolarCoordAngle = currentBaseAngle


    #end get polar coordinates


def get_radius_in_horizontal_plane_to_cartesian_end_effector_position_using_2d_revolute_revolute_forward_kinematics(upperArmAngle, lowerArmAngle, upperArmLength, lowerArmLength):
    #the equation for radius is determined using forward kinematics, just uses socahtoa rules, namely coa here.
    radius = ( math.cos(upperArmAngle) * upperArmLength ) + ( math.cos(upperArmAngle + lowerArmAngle) * lowerArmLength )
    return radius

"""


if __name__ == '__main__':

    ik = DobotInverseKinematics()
    commandFlag = True
    while(commandFlag):
        x = input('Enter x: ')
        y = input('Enter y: ')
        z = input('Enter z: ')
        angles = ik.convert_cartesian_coordinate_to_arm_angles(float(x),float(y),float(z))

        if(angles[0] != -999):
            print('\nFor the point (' + str(x) + ' , ' + str(y) + ' , ' + str(z) + ') , the angles are:' )
            print('Base Angle: ' + str(angles[0]) )
            print('Upper Arm Angle: ' + str(angles[1]) )
            print('Lower Arm Angle: ' + str(angles[2]))
            print('\n')
        else:
            print('Invalid coordinate: Out of arm\'s range.')
            print('\n')

        s = input('quit?')
        if (s == 'y'):
            commandFlag = False
        print('\n')

