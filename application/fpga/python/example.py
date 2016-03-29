#! /usr/bin/env python
'''
Simple demostration of open-dobot.

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

from Dobot import Dobot

dobot = Dobot('/dev/tty.usbmodem1421', 115200)
dobot.Open()
successes = 0
i = 0
while True:
	ret = dobot.isReady()
	if ret[0] and ret[1]:
		successes += 1
	if successes > 10:
		print "Dobot ready!"
		break
	if i > 100:
		raise Exception('Comm problem')

print dobot.GetAccelerometers()

# Magic number corresponding to frequency in Hz.
# dict: {frequency: code}
# where frequency is not used, the code is some magic number yet to be decoded, 
freq = {
	   0: 0x0242f000,
	  50: 0xf885e000,
	 250: 0xf9618000,
	 400: 0xc42f0000,
	 600: 0x834500d5,
	 800: 0x885e0000,
	 950: 0x53660053,
	1150: 0x572a0084,
	1300: 0xb8d200a3,
	1450: 0x1ac20086,
	1500: 0x98820055,
	1800: 0xfc6c00c7
}

def execute(keys1, keys2, keys3, direction1, direction2, direction3):
	for key1, key2, key3 in zip(keys1, keys2, keys3):
		code1 = freq[key1]
		code2 = freq[key2]
		code3 = freq[key3]
		for i in range(0, 4):
			ret = (1, 0)
			# Check for return from Arduino to make sure the command was queued.
			# See function desciption for more details.
			while not ret[1]:
				ret = dobot.Steps(code1, code2, code3, direction1, direction2, direction3)

increasing = sorted(freq.keys())
decreasing = sorted(freq.keys(), reverse=True)
execute(increasing, [], increasing, 0, 0, 0)
while True:
	execute(decreasing, increasing, decreasing, 0, 0, 0)
	execute(increasing, decreasing, increasing, 1, 0, 1)
	execute(decreasing, increasing, decreasing, 1, 1, 1)
	execute(increasing, decreasing, increasing, 0, 1, 0)
