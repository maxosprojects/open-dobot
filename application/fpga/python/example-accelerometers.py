#! /usr/bin/env python

'''
This example continuously reports accelerometers.
Refer to _SwitchToAccelerometerReportMode function in DobotDriver.py for details on
how to enable reporting mode.
'''

from DobotDriver import DobotDriver

driver = DobotDriver('/dev/tty.usbmodem1421', 115200)
driver.Open()

while True:
	print driver.GetAccelerometers()
