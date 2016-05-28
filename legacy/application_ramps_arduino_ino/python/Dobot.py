'''
open-dobot driver for RAMPS version.

Find improved FPGA version, driver and SDK at https://github.com/maxosprojects/open-dobot

Author: maxosprojects (March 18 2016)
Additional Authors: <put your name here>

License: MIT
'''

import serial
import time
import threading
import sys

_max_trys = 3

CMD_READY = 0
CMD_STEPS = 1
CMD_EXEC_QUEUE = 2

class Dobot:
	def __init__(self, comport, rate):
		self._lock = threading.Lock()
		self._comport = comport
		self._rate = rate
		self._port = None
		self._crc = 0xffff

	def Open(self, timeout=0.1):
		self._port = serial.Serial(self._comport, baudrate=self._rate, timeout=timeout, interCharTimeout=0.01)

	def Close(self):
		self._port.close()

	def _crc_clear(self):
		self._crc = 0xffff

	def _crc_update(self, data):
		self._crc = self._crc ^ (data << 8)
		for bit in range(0, 8):
			if (self._crc&0x8000) == 0x8000:
				self._crc = ((self._crc << 1) ^ 0x1021)
			else:
				self._crc = self._crc << 1

	def _readchecksumword(self):
		data = self._port.read(2)
		if len(data)==2:
			crc = (ord(data[0])<<8) | ord(data[1])
			return (1,crc)	
		return (0,0)

	def _readbyte(self):
		data = self._port.read(1)
		if len(data):
			val = ord(data)
			self._crc_update(val)
			return (1,val)	
		return (0,0)

	def _readlong(self):
		val1 = self._readbyte()
		if val1[0]:
			val2 = self._readbyte()
			if val2[0]:
				val3 = self._readbyte()
				if val3[0]:
					val4 = self._readbyte()
					if val4[0]:
						return (1,val1[1]<<24|val2[1]<<16|val3[1]<<8|val4[1])
		return (0,0)	

	def _readslong(self):
		val = self._readlong()
		if val[0]:
			if val[1]&0x80000000:
				return (val[0],val[1]-0x100000000)
			return (val[0],val[1])
		return (0,0)

	def _read1(self, cmd):
		trys = _max_trys
		while trys:
			self._port.flushInput()
			self._sendcommand(cmd)
			val1 = self._readbyte()
			if val1[0]:
				crc = self._readchecksumword()
				if crc[0]:
					if self._crc&0xFFFF!=crc[1]&0xFFFF:
						raise Exception('crc differs', self._crc, crc)
						return (0,0)
					return (1,val1[1])
			trys -= 1
		# raise Exception("couldn't get response in time for", _max_trys, 'times')
		return (0,0)

	def _read4(self, cmd):
		trys = _max_trys
		while trys:
			self._port.flushInput()
			self._sendcommand(cmd)
			val1 = self._readlong()
			if val1[0]:
				crc = self._readchecksumword()
				if crc[0]:
					if self._crc&0xFFFF!=crc[1]&0xFFFF:
						return (0,0)
					return (1,val1[1])
			trys -= 1
		return (0,0)

	def _read4_1(self, cmd):
		trys = _max_trys
		while trys:
			self._port.flushInput()
			self._sendcommand(cmd)
			val1 = self._readslong()
			if val1[0]:
				val2 = self._readbyte()
				if val2[0]:
					crc = self._readchecksumword()
					if crc[0]:
						if self._crc&0xFFFF!=crc[1]&0xFFFF:
							return (0,0)
						return (1,val1[1],val2[1])
			trys -= 1
		return (0,0)

	def _writebyte(self, val):
		self._crc_update(val&0xFF)
		self._port.write(chr(val&0xFF))

	def _writeword(self, val):
		self._writebyte((val>>8)&0xFF)
		self._writebyte(val&0xFF)

	def _writelong(self, val):
		self._writebyte((val>>24)&0xFF)
		self._writebyte((val>>16)&0xFF)
		self._writebyte((val>>8)&0xFF)
		self._writebyte(val&0xFF)

	def _writechecksum(self):
		self._writeword(self._crc&0xFFFF)
		val = self._readbyte()
		if val[0]:
			return True
		return False

	def _sendcommand(self, command):
		self._crc_clear()
		self._crc_update(command)
		self._port.write(chr(command))
		return

	def _write0(self, cmd):
		trys = _max_trys
		while trys:
			self._sendcommand(cmd)
			if self._writechecksum():
				return True
			trys = trys - 1
		return False

	def _write1(self, cmd, val):
		trys = _max_trys
		while trys:
			self._sendcommand(cmd)
			self._writebyte(val)
			if self._writechecksum():
				return True
			trys = trys - 1
		return False

	def _write2(self, cmd, val):
		trys = _max_trys
		while trys:
			self._sendcommand(cmd)
			self._writeword(val)
			if self._writechecksum():
				return True
			trys = trys - 1
		return False

	def _write4(self, cmd, val):
		trys = _max_trys
		while trys:
			self._sendcommand(cmd)
			self._writelong(val)
			if self._writechecksum():
				return True
			trys = trys - 1
		return False

	def _write14(self, cmd, val1, val2):
		trys = _max_trys
		while trys:
			self._sendcommand(cmd)
			self._writebyte(val1)
			self._writelong(val2)
			if self._writechecksum():
				return True
			trys = trys - 1
		return False

	def _write11121read1(self, cmd, val1, val2, val3, val4, val5):
		trys = _max_trys
		while trys:
			self._sendcommand(cmd)
			self._writebyte(val1)
			self._writebyte(val2)
			self._writebyte(val3)
			self._writeword(val4)
			self._writebyte(val5)
			self._writeword(self._crc&0xFFFF)
			self._port.flushInput()
			self._crc_clear()
			ret = self._readbyte()
			if ret[0]:
				crc = self._readchecksumword()
				if crc[0]:
					if self._crc&0xFFFF!=crc[1]&0xFFFF:
						raise Exception('crc differs', self._crc, crc)
						return (0,0)
					return (1,ret[1])
			trys = trys - 1
		return (0,0)

	def Steps(self, j1, j2, j3, ticks, j1dir, j2dir, j3dir, deferred):
		'''
		Adds a command to the controller queue.
		Controller runs a timer that counts ticks (around 3.47kHz). "Steps" command specifies
		a scaler for each joint (joint1 - base, and so on, according to the official docs).
		On every "ticks modulo scaler == 0" the STEP pin (on the stepper driver) is toggled.
		So, if scaler=1, then the STEP pin is toggled at around 3.47kHz frequency. If scaler
		is 10, then STEP pin is toggled at around 347Hz.
		
		The "Steps" command also specifies the number of ticks to run the command for, the
		state for DIR pin (on the stepper driver) to indicate which direction the motor will
		turn, and the deferred flag to indicate whether the command should not be executed until
		the "ExecQueue" command is issued (for precise and smooth trajectories built of many
		short-running commands).

		@param j1 - joint1 scaler
		@param j2 - joint2 scaler
		@param j3 - joint3 scaler
		@param ticks - number of ticks to run the command for
		@param j1dir - direction for joint1
		@param j2dir - direction for joint2
		@param j3dir - direction for joint3
		@param deferred - defer execution of this command and all commands issued after this until
						the "ExecQueue" command is issued.
		@return Returns a tuple where the first element tells whether the command has been successfully
		received (0 - yes, 1 - timed out), and the second element tells whether the command was added
		to the controller's command queue (1 - added, 0 - not added, as the queue was full).
		'''
		control = ((j1dir & 0x01) << 1) | ((j2dir & 0x01) << 2) | ((j3dir & 0x01) << 3);
		if deferred:
			control |= 0x01
		self._lock.acquire()
		result = self._write11121read1(CMD_STEPS, j1, j2, j3, ticks, control)
		self._lock.release()
		return result

	def ExecQueue(self):
		'''
		Executes deferred commands.
		'''
		self._lock.acquire()
		result = self._write0(CMD_EXEC_QUEUE)
		self._lock.release()
		return result

	def isReady(self):
		'''
		Checks whether the controller is up and running.
		'''
		self._lock.acquire()
		result = self._read1(CMD_READY)
		self._lock.release()
		# Check for magic number.
		# return [result[0], result[1] == 0x40]
		return [result[0], result[1]]

	def reset(self):
#		self._lock.acquire()
		i = 0
		while i < 5:
			self._port.flushInput()
			self._port.read(1)
			i += 1
		self._crc_clear()
#		self._lock.release()
