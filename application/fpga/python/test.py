#! /usr/bin/env python

from Dobot import Dobot

dobot = Dobot('/dev/tty.usbmodem1421', 115200)
dobot.Open()

def reverseBits(val):
	rev = 0
	for i in range(0, 32):
		rev |= (((val & 0xFFFFFFFF) >> i) & 0x00000001) << (31 - i)
	return rev & 0xFFFFFFFF

def convertFreq(val):
	return reverseBits(25000000 / val)

while True:
	ret = dobot.Steps(convertFreq(504), 0, 0, 0, 0, 0)
