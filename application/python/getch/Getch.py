
import sys

class _Getch:
    """Gets a single character from standard input. Does not echo to the screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()

class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()

getch = _Getch()

'''
# Example:

from getch import getch

from dobot import DobotDriver
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
		print("Dobot ready!")
		break
	if i > 100:
		raise Exception('Comm problem')

gripper = 480
toolRotation = 0

stepOne = driver.stepsToCmdVal(1)

left = 68
right = 67

def do_nothing():
	pass

def doSteps(dir):
	ret = (0, 0)
	while not ret[1]:
		ret = driver.Steps(stepOne, 0, 0, dir, 0, 0, gripper, toolRotation)

def doLeft():
	print 'Moving left'
	doSteps(1)

def doRight():
	print 'Moving right'
	doSteps(0)

switcher = {}
switcher[left] = (doLeft,)
switcher[right] = (doRight,)

def move(ch):
	global switcher
	func, = switcher.get(ch, (do_nothing,))
	func()

while True:
	ch = ord(getch())
	# To process Ctrl-C
	# If not do that then the only way to stop the programm is to kill the process.
	if ch == 3:
		exit(0)
	# Don't process further if no escape character - so arrow keys in this case.
	if ch == 27:
		ch = ord(getch())
		# Arrow key.
		if ch == 91:
			ch = ord(getch())
			move(ch)
'''
