from Dobot import Dobot
import time
import math
# import numpy as np
# import matplotlib.pyplot as plt

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

while True:
	print dobot.Steps(0xf885e000, 0, 0, 0, 0, 0)
	# time.sleep(5)
