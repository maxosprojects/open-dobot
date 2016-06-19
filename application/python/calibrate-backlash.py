#! /usr/bin/env python
'''
Allows to visually calibrate motor reduction gear backlash.

The backlash is hardcoded in SDK. Use only if you know what you're doing.
'''

from dobot import DobotDriver
from getch import getch
import time

# driver = DobotDriver('COM4')
driver = DobotDriver('/dev/tty.usbmodem1421')
driver.Open()
successes = 0
i = 0
while True:
	ret = driver.isReady()
	if ret[0] and ret[1]:
		successes += 1
	if successes > 10:
		print("Dobot ready!")
		break
	if i > 100:
		raise Exception('Comm problem')

def execute(code1, code2, code3, direction1, direction2, direction3):
	code1 = driver.stepsToCmdVal(code1)
	code2 = driver.stepsToCmdVal(code2)
	code3 = driver.stepsToCmdVal(code3)
	ret = (0, 0)
	while not ret[1]:
		ret = driver.Steps(code1, code2, code3, direction1, direction2, direction3, 0, 0)
	driver.Wait(0.5)

def calibrate(steps1, steps2, steps3):
	for i in range(5):
		execute(steps1, steps2, steps3, 0, 0, 0)
		execute(steps1, steps2, steps3, 1, 1, 1)
	time.sleep(2)

def question(q):
	inp = ''
	while inp != 'y' and inp != 'n':
		print(q + ' (y/n)? ')
		inp = getch()
		ch = ord(inp)
		if ch == 3:
			print('Ctrl-C pressed. Bye')
			exit(0)
		if inp == 'r':
			return None
	return inp == 'y'

print('Please press "y" or "n" to the following questions')
print('To repeat last iteration press "r"')

if not question('Do you want to start calibration'):
	print('Bye')
	exit(0)

print('Calibrating Joint 1 (base)')

steps = 50
lastSteps = steps
lastLow = 0
lastHigh = steps
while True:
	print('')
	print('Calibrating with number of steps ' + str(steps))
	calibrate(steps, 0, 0)
	ret = question('Did you see joint move')
	if ret == None:
		continue
	if ret:
		lastHigh = steps
	else:
		lastLow = steps
	lastSteps = steps
	steps = int((lastHigh + lastLow) / 2)
	if lastSteps == steps:
		break

print('')
print('Backlash is ' + str(steps))

