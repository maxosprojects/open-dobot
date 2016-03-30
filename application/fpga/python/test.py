#! /usr/bin/env python

from DobotDriver import DobotDriver

driver = DobotDriver('/dev/tty.usbmodem1421', 115200)
driver.Open()

def reverseBits(val):
	rev = 0
	for i in range(0, 32):
		rev |= (((val & 0xFFFFFFFF) >> i) & 0x00000001) << (31 - i)
	return rev & 0xFFFFFFFF

def convertFreq(val):
	if val == 0:
		return 0x0242f000;
	return reverseBits(25000000 / val)

print convertFreq(6300)
print reverseBits(convertFreq(6300))

while True:
	ret = driver.Steps(0, convertFreq(2250), convertFreq(6300), 0, 0, 1)
