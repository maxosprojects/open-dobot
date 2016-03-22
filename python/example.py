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

def accel(start, end, defer):
	step = 1
	# distance = abs(end - start)
	if start > end:
		step = -1
	# x = range(start, end, step)
	# y = np.array([])
	for i in range(start, end, step):
		# scale = -1.0 * math.pow(0.9, i) + 1
		# scale = float(i) / float(distance)
		done = False
		while not done:
			# ret = dobot.Steps(255 - i,  255 - i, 255 - i, int(200 * scale), 0, 0, 0, defer)
			ret = dobot.Steps(255 - i,  255 - i, 255 - i, 200, 0, 0, 0, defer)
			# print ret, i
			done = ret[1] == 1
		# y = np.append(y, scale)
	# print x
	# print y
	# line, = plt.plot(x, y, '--', linewidth=2)
	# plt.show()

accel(1, 254, False)
# dobot.ExecQueue()
# print 'Executing queue'
dobot.Steps(1, 1, 1, 2000, 0, 0, 0, False)
print 'Adding fast section'
accel(254, 1, False)
