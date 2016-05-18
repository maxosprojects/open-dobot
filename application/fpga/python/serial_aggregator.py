"""
open-dobot serial aggregator.

This is a workaround to send data in bursts on systems that have slow API
used by pyserial (e.g. Windows).

Author: maxosprojects (May 17 2016)
Additional Authors: <put your name here>

Version: 1.1.0

License: MIT
"""

class serial_aggregator:
	def __init__(self, ser):
		self._ser = ser
		self._buf = bytearray()

	def write(self, data):
		self._buf.extend(data)

	def read(self, size):
		return self._ser.read(size)

	def flushInput(self):
		pass

	def flush(self):
		pass

	def send(self):
		self._ser.write(self._buf)
		self._buf = bytearray()
