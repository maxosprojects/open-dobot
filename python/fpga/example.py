from Dobot import Dobot
import time
import math

dobot = Dobot('/dev/tty.usbmodem1421', 115200)
dobot.Open()
time.sleep(1)
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

# frequency: (code, iterations)
# where frequency is not used, the code is some magic code yet to be decoded, 
# iterations - how many commands will be sent. There are 50 commands/second dobot
# executes (50Hz). So if 50 commands are sent dobot will execute for a second.
freq = {
	0: (0x00000000, 2),
	50: (0xf885e000, 5),
	250: (0xf9618000, 10),
	400: (0xc42f0000, 10),
	600: (0x834500d5, 10)
}

def execute(keys, direction):
	for key in keys:
		tup = freq[key]
		for i in range(0, tup[1]):
			dobot.Steps(tup[0], 0, 0, direction, 0, 0)

dir = 0
while True:
	execute(sorted(freq.keys()), dir)
	execute(sorted(freq.keys(), reverse=True), dir)
	if dir:
		dir = 0
	else:
		dir = 1
