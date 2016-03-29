
'''
This example continuously reports accelerometers.
Refer to _SwitchToAccelerometerReportMode function in Dobot.py for details on
how to enable reporting mode and that way.
'''

from Dobot import Dobot
import time
import math

dobot = Dobot('/dev/tty.usbmodem1421', 115200)
dobot.Open()

while True:
	print dobot.GetAccelerometers()
