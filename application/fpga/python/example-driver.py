#! /usr/bin/env python
'''
Simple demostration of open-dobot. Low-level control via driver.

The commands are queued in Arduino, hence when you stop this example dobot will continue
to execute until the queue (200 commands) is empty.

There are 50 commands/second dobot executes (50Hz). So if you send only 50 commands dobot
will execute for exactly one second.

Number of steps is determined by the frequency and the execution window (20ms = 0.02s).

So, if you send only one command "50Hz" then dobot makes exacly one step and stops.

number_of_steps = 50Hz * 20ms = 50 * 0.02 = 1

For 600Hz:
number_of_steps = 600Hz * 20ms = 600 * 0.02 = 12

'''

from DobotDriver import DobotDriver
import time

driver = DobotDriver('/dev/tty.usbmodem1421')
driver.Open()
successes = 0
i = 0
while True:
	ret = driver.isReady()
	if ret[0] and ret[1]:
		successes += 1
	if successes > 10:
		print "Dobot ready!"
		break
	if i > 100:
		raise Exception('Comm problem')

print 'Accelerometer data returned', driver.GetAccelerometers()


steps = driver.stepsToCmdVal(275)

while True:
	driver.SetCounters(-550, 3, 5)
	ret = driver.Steps(steps, 0, steps, 1, 0, 1)
	time.sleep(0.5)
	print driver.GetCounters()

freq = [
	   0,
	  50,
	 250,
	 400,
	 600,
	 800,
	 950,
	1150,
	1300,
	1450,
	1500,
	1800
]

def execute(keys1, keys2, keys3, direction1, direction2, direction3):
	for key1, key2, key3 in zip(keys1, keys2, keys3):
		code1 = driver.freqToCmdVal(key1)
		code2 = driver.freqToCmdVal(key2)
		code3 = driver.freqToCmdVal(key3)
		for i in range(0, 4):
			ret = (1, 0)
			# Check for return from Arduino to make sure the command was queued.
			# See function desciption for more details.
			while not ret[1]:
				ret = driver.Steps(code1, code2, code3, direction1, direction2, direction3)

increasing = freq
decreasing = sorted(freq, reverse=True)
execute(increasing, [], increasing, 0, 0, 0)
while True:
	execute(decreasing, increasing, decreasing, 0, 0, 0)
	execute(increasing, decreasing, increasing, 1, 0, 1)
	execute(decreasing, increasing, decreasing, 1, 1, 1)
	execute(increasing, decreasing, increasing, 0, 1, 0)
