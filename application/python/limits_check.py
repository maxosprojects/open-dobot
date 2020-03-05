import math

lengthRearArm = 135.0
lengthForearm = 160.0
# Distance from joint3 to the center of the tool mounted on the end effector.
distanceTool = 50.9
heightFromBase = 80.0 + 23.0

maxDistance = lengthRearArm + lengthForearm

speed = None
acceleration = None

print('Maximum arm stretch', maxDistance)


def MoveWithSpeed(x, y, z, ignore1, ignore2):
    radius = hypothenuse(x, y) + distanceTool
    height = z - heightFromBase
    distance = hypothenuse(radius, height)
    print((x, y, z), 'distance', distance)


def hypothenuse(a, b):
    return math.sqrt(math.pow(a, 2) + math.pow(b, 2))


MoveWithSpeed(210.9, 0, 0, speed, acceleration)
MoveWithSpeed(210.9, 0, 238, speed, acceleration)
MoveWithSpeed(210.9, 150, 238, speed, acceleration)

MoveWithSpeed(0, 150, 238, 50, acceleration)

MoveWithSpeed(210.9, -150, 238, 50, acceleration)
MoveWithSpeed(210.9, 0, 238, speed, acceleration)
