from Dobot import Dobot
import time
import math

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

# frequency: (code, iterations)
# where frequency is not used, the code is some magic code yet to be decoded, 
# iterations - how many commands will be sent. There are 50 commands/second dobot
# executes (50Hz). So if 50 commands are sent dobot will execute for a second.
freq = {
	   0: (0x0242f000, 2),
	  50: (0xf885e000, 5),
	 250: (0xf9618000, 10),
	 400: (0xc42f0000, 10),
	 600: (0x834500d5, 10),
	 800: (0x885e0000, 10),
	 950: (0x53660053, 10),
	1150: (0x572a0084, 10),
	1300: (0xb8d200a3, 10),
	1450: (0x1ac20086, 10)
}

def execute(keys1, keys2, direction1, direction2):
	for key1, key2 in zip(keys1, keys2):
		code1 = freq[key1]
		code2 = freq[key2]
		for i in range(0, 5):
			dobot.Steps(code1[0], code2[0], 0, direction1, direction2, 0)

increasing = sorted(freq.keys())
decreasing = sorted(freq.keys(), reverse=True)
execute(increasing, [], 0, 0)
while True:
	execute(decreasing, increasing, 0, 0)
	execute(increasing, decreasing, 1, 0)
	execute(decreasing, increasing, 1, 1)
	execute(increasing, decreasing, 0, 1)
