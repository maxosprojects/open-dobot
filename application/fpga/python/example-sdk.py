#! /usr/bin/env python
'''
Simple demostration of open-dobot. High-level control via SDK.

It is assumed that no end effectors are installed on the arm (no gripper or sucker, etc.)
as end effector would crash into the desk Dobot stands on.

Refer to SDK to find the expected initial arm configuration.

Each move is split into a series of commands to send via driver to FPGA. Each command is
executed for 20ms. Dobot, thus, executes 50 commands per second.

The commands are queued in Arduino, hence when you stop this example dobot will continue
to execute until the queue (200 commands) is empty.

'''

from DobotSDK import Dobot

dobot = Dobot('/dev/tty.usbmodem1421', debug=True)

dobot.moveTo(160.0, 0.0, 60.0, 1)
print '======================================'
for i in range(3):
	dobot.moveTo(160.0, 60.0, 60.0, 1)
	print '======================================'
	dobot.moveTo(220.0, 60.0, 60.0, 1)
	print '======================================'
	dobot.moveTo(220.0, -60.0, 60.0, 1)
	print '======================================'
	dobot.moveTo(160.0, -60.0, 60.0, 1)
	print '======================================'
dobot.moveTo(160.0, 0.0, 60.0, 1)
print '======================================'
dobot.moveTo(160.0, 0.0, 215.0, 1)
