"""
open-dobot driver.

Implements driver to open firmware that controls Dobot FPGA.
Abstracts communication protocol, CCITT CRC and commands sent to FPGA.
Find firmware and SDK at https://github.com/maxosprojects/open-dobot

Author: maxosprojects (March 18 2016)
Additional Authors: <put your name here>

Version: 1.2.2

License: MIT
"""

import serial
import threading
import time
from serial import SerialException
import math
import sys

# Workaround to support Python 2/3
if sys.version_info > (3,):
	long = int

_max_trys = 1

CMD_READY = 0
CMD_STEPS = 1
CMD_EXEC_QUEUE = 2
CMD_GET_ACCELS = 3
CMD_SWITCH_TO_ACCEL_REPORT_MODE = 4
CMD_CALIBRATE_JOINT = 5
CMD_EMERGENCY_STOP = 6
CMD_SET_COUNTERS = 7
CMD_GET_COUNTERS = 8
CMD_LASER_ON = 9
CMD_PUMP_ON = 10
CMD_VALVE_ON = 11
CMD_BOARD_VERSION = 12

piToDegrees = 180.0 / math.pi
halfPi = math.pi / 2.0

class DobotDriver:
	def __init__(self, comport, rate=115200):
		self._lock = threading.Lock()
		self._comport = comport
		self._rate = rate
		self._port = None
		self._crc = 0xffff
		self.FPGA = 0
		self.RAMPS = 1
		self._toolRotation = 0
		self._gripper = 480

	def Open(self, timeout=0.025):
		try:
			self._port = serial_aggregator(serial.Serial(self._comport, baudrate=self._rate, timeout=timeout, interCharTimeout=0.1))
			# self._port = serial.Serial(self._comport, baudrate=self._rate, timeout=timeout, interCharTimeout=0.1)

			# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			# s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
			# s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 2)
			# s.connect(("localhost", 5555))
			# self._port = serial2socket(s)

			# Have to wait for Arduino initialization to finish, or else it doesn't boot.
			time.sleep(2)
		except SerialException as e:
			print(e)
			exit(1)

		ret = (0, 0)
		i = 200
		while not ret[0] and i > 0:
			ret = self.BoardVersion()
			i -= 1
		if i == 0:
			print("Cannot get board version. Giving up")
			exit(1)

		self._ramps = bool(ret[1])
		if self._ramps:
			print("Board: RAMPS")
		else:
			print("Board: FPGA")

		if self._ramps:
			self._stepCoeff = 20000
			self._stopSeq = self.reverseBits32(0)
		else:
			self._stepCoeff = 500000
			self._stopSeq = 0x0242f000
		self._stepCoeffOver2 = self._stepCoeff / 2
		self._freqCoeff = self._stepCoeff * 25

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
			arr = bytearray(data)
			crc = (arr[0]<<8) | arr[1]
			return (1,crc)
		return (0,0)

	def _readbyte(self):
		data = self._port.read(1)
		if len(data):
			val = bytearray(data)[0]
			self._crc_update(val)
			return (1,val)	
		return (0,0)

	def _readword(self):
		val1 = self._readbyte()
		if val1[0]:
			val2 = self._readbyte()
			if val2[0]:
				return (1,val1[1]<<8|val2[1])
		return (0,0)

	def _readsword(self):
		val = self._readword()
		if val[0]:
			if val[1]&0x8000:
				return (val[0],val[1]-0x10000)
			return (val[0],val[1])
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
		return self._read(cmd, [self._readbyte])

	def _read22(self, cmd):
		return self._read(cmd, [self._readword,
								self._readword])

	def _reads22(self, cmd):
		return self._read(cmd, [self._readsword,
								self._readsword])

	def _reads222222(self, cmd):
		return self._read(cmd, [self._readsword,
								self._readsword,
								self._readsword,
								self._readsword,
								self._readsword,
								self._readsword])

	def _read4(self, cmd):
		return self._read(cmd, [self._readlong])

	def _read41(self, cmd):
		return self._read(cmd, [self._readslong,
								self._readbyte])

	def _reads444(self, cmd):
		return self._read(cmd, [self._readslong,
								self._readslong,
								self._readslong])

	def _read(self, cmd, read_commands=list()):
		trys = _max_trys
		while trys:
			self._sendcommand(cmd)
			self._writechecksum()
			self._port.send()

			ret = [1]
			for c in read_commands:
				val = c()
				if not val[0]:
					return tuple([0] * (len(read_commands) + 1))
				ret.append(val[1])

			crc = self._readchecksumword()
			if crc[0]:
				if self._crc&0xFFFF == crc[1]&0xFFFF:
					return tuple(ret)
			trys -= 1
		return tuple([0] * (len(read_commands) + 1))

	def _writebyte(self, val):
		self._crc_update(val&0xFF)
		self._port.write(bytearray([val&0xFF]))

	def _writeword(self, val):
		self._writebyte((val>>8)&0xFF)
		self._writebyte(val&0xFF)

	def _writelong(self, val):
		self._writebyte((val>>24)&0xFF)
		self._writebyte((val>>16)&0xFF)
		self._writebyte((val>>8)&0xFF)
		self._writebyte(val&0xFF)

	def _writechecksum(self):
		self._port.write(bytearray([(self._crc>>8)&0xFF]))
		self._port.write(bytearray([self._crc&0xFF]))

	def _sendcommand(self, command):
		self._crc_clear()
		self._writebyte(command)

	def _write(self, cmd, write_commands=list()):
		trys = _max_trys
		while trys:
			self._sendcommand(cmd)

			for c in write_commands:
				c[0](c[1])

			self._writechecksum()
			self._port.send()
			crc = self._readchecksumword()
			if crc[0]:
				if self._crc&0xFFFF == crc[1]&0xFFFF:
					return True
			trys -= 1
		return False

	def _write0(self, cmd):
		return self._write(cmd)

	def _write1(self, cmd, val):
		return self._write(cmd, [(self._writebyte, val)])

	def _write2(self, cmd, val):
		return self._write(cmd, [(self._writeword, val)])

	def _write4(self, cmd, val):
		return self._write(cmd, [(self._writelong, val)])

	def _write14(self, cmd, val1, val2):
		return self._write(cmd, [(self._writebyte, val1), (self._writelong, val2)])

	def _write14411(self, cmd, val1, val2, val3, val4):
		return self._write(cmd, [(self._writelong, val1),
									(self._writelong, val2),
									(self._writebyte, val3),
									(self._writebyte, val4)])

	def _write444(self, cmd, val1, val2, val3):
		return self._write(cmd, [(self._writelong, val1),
									(self._writelong, val2),
									(self._writelong, val3)])

	def _write_read(self, cmd, write_commands):
		tries = _max_trys
		while tries:
			self._sendcommand(cmd)

			for c in write_commands:
				c[0](c[1])

			self._writechecksum()
			self._port.send()
			ret = self._readbyte()
			if ret[0]:
				crc = self._readchecksumword()
				if crc[0]:
					if self._crc & 0xFFFF != crc[1] & 0xFFFF:
						# raise Exception('crc differs', self._crc, crc)
						return (0, 0)
					return (1, ret[1])
			tries -= 1
		return (0, 0)

	def _write1read1(self, cmd, val1):
		return self._write_read(cmd, [(self._writebyte, val1)])

	def _write11121read1(self, cmd, val1, val2, val3, val4, val5):
		return self._write_read(cmd, [(self._writebyte, val1),
									(self._writebyte, val2),
									(self._writebyte, val3),
									(self._writeword, val4),
									(self._writebyte, val5)])

	def _write14441read1(self, cmd, val1, val2, val3, val4):
		return self._write_read(cmd, [(self._writelong, val1),
									(self._writelong, val2),
									(self._writelong, val3),
									(self._writebyte, val4)])

	def _write1444122read1(self, cmd, val1, val2, val3, val4, val5, val6):
		return self._write_read(cmd, [(self._writelong, val1),
									(self._writelong, val2),
									(self._writelong, val3),
									(self._writebyte, val4),
									(self._writeword, val5),
									(self._writeword, val6)])

	def reverseBits32(self, val):
		### return long(bin(val&0xFFFFFFFF)[:1:-1], 2)
		# return int('{0:032b}'.format(val)[::-1], 2)
		# Not reversing bits in bytes any more as SPI switched to LSB first.
		# But still need to reverse bytes places.
		return ((val & 0x000000FF) << 24) | ((val & 0x0000FF00) << 8) | ((val & 0x00FF0000) >> 8) | ((val & 0xFF000000) >> 24)

	def reverseBits16(self, val):
		# Not reversing bits in bytes any more as SPI switched to LSB first.
		# But still need to reverse bytes places.
		return ((val & 0x00FF) << 8) | ((val & 0xFF00) >> 8)

	def freqToCmdVal(self, freq):
		'''
		Converts stepping frequency into a command value that dobot takes.
		'''
		if freq == 0:
			return self._stopSeq
		return self.reverseBits32(long((self._freqCoeff) / freq))

	def stepsToCmdVal(self, steps):
		'''
		Converts number of steps for dobot to do in 20ms into a command value that dobot
		takes to set the stepping frequency.
		'''
		if steps == 0:
			return self._stopSeq
		return self.reverseBits32(long(self._stepCoeff / steps))

	def stepsToCmdValFloat(self, steps):
		'''
		Converts number of steps for dobot to do in 20ms into a command value that dobot
		takes to set the stepping frequency.

		@param steps - float number of steps; float to minimize error and have finer control
		@return tuple (command_value, leftover), where leftover is the fractioned steps that don't fit
				into 20ms interval a command runs for
		'''
		if abs(steps) < 0.01:
			return (self._stopSeq, 0, 0.0)
		# "round" makes leftover negative in certain cases and causes backlash compensation to oscillate.
		# actualSteps = long(round(steps))
		actualSteps = long(steps)
		if actualSteps == 0:
			return (self._stopSeq, 0, steps)
		val = long(self._stepCoeff / actualSteps)
		actualSteps = long(self._stepCoeff / val)
		if val == 0:
			return (self._stopSeq, 0, steps)
		return (self.reverseBits32(val), actualSteps, steps - actualSteps)

	def accelToAngle(self, val, offset):
		return self.accelToRadians(val, offset) * piToDegrees

	def accel3DXToAngle(self, x, y, z):
		return self.accel3DXToRadians(x, y, z) * piToDegrees

	def accelToRadians(self, val, offset):
		try:
			return math.asin(float(val - offset) / 493.56)
		except ValueError:
			return halfPi

	def accel3DXToRadians(self, x, y, z):
		try:
			xf = float(x)
			yf = float(y)
			zf = float(z)
			return math.atan2(xf, math.sqrt(yf * yf + zf * zf))
		except ValueError:
			return halfPi

	def CalibrateJoint(self, joint, forwardCommand, backwardCommand, direction, pin, pinMode, pullup):
		'''
		Initiates joint calibration procedure using a limit switch/photointerrupter. Effective immediately.
		Current command buffer is cleared.
		Cancel the procedure by issuing EmergencyStop() is necessary.
		
		@param joint - which joint to calibrate: 1-3
		@param forwardCommand - command to send to the joint when moving forward (towards limit switch);
				use freqToCmdVal()
		@param backwardCommand - command to send to the joint after hitting  (towards limit switch);
				use freqToCmdVal()
		@param direction - direction to move joint towards limit switch/photointerrupter: 0-1
		@param pin - firmware internal pin reference number that limit switch is connected to;
					refer to dobot.h -> calibrationPins
		@param pinMode - limit switch/photointerrupter normal LOW = 0, normal HIGH = 1
		@param pullup - enable pullup on the pin = 1, disable = 0
		@return True if command succesfully received, False otherwise.
		'''
		if 1 > joint > 3:
			return False
		control = ((pinMode & 0x01) << 4) | ((pullup & 0x01) << 3) | ((direction & 0x01) << 2) | ((joint - 1) & 0x03)
		self._lock.acquire()
		result = self._write14411(CMD_CALIBRATE_JOINT, forwardCommand, backwardCommand, pin, control)
		self._lock.release()
		return result

	def EmergencyStop(self):
		'''
		Stops the arm in case of emergency. Clears command buffer and cancels calibration procedure.

		@return True if command succesfully received, False otherwise.
		'''
		self._lock.acquire()
		result = self._write0(CMD_EMERGENCY_STOP)
		self._lock.release()
		return result

	def Steps(self, j1, j2, j3, j1dir, j2dir, j3dir, servoGrab, servoRot, deferred=False):
		'''
		Adds a command to the controller's queue to execute on FPGA.
		@param j1 - joint1 subcommand
		@param j2 - joint2 subcommand
		@param j3 - joint3 subcommand
		@param j1dir - direction for joint1: 0-1
		@param j2dir - direction for joint2: 0-1
		@param j3dir - direction for joint3: 0-1
		@param servoGrab - servoGrab position (gripper): 0x00d0-0x01e0 (or 208-480 decimal)
		@param servoRot - servoRot position (tool rotation): 0x0000-0x0400 (or 0-1024 decimal)
		@param deferred - defer execution of this command and all commands issued after this until
						the "ExecQueue" command is issued. Currently ignored.
		@return Returns a tuple where the first element tells whether the command has been successfully
		received (1 - received, 0 - timed out), and the second element tells whether the command was added
		to the controller's command queue (1 - added, 0 - not added, as the queue was full).
		'''
		control = (j1dir & 0x01) | ((j2dir & 0x01) << 1) | ((j3dir & 0x01) << 2);
		# if deferred:
		# 	control |= 0x01
		self._lock.acquire()
		if servoGrab > 480:
			servoGrab = 480
		elif servoGrab < 208:
			servoGrab = 208
		if servoRot > 1024:
			servoRot = 1024
		elif servoRot < 0:
			servoRot = 0

		if self._ramps:
			servoRot *= 2;
			servoRot += 2000;
			servoGrab *= 2;
			servoGrab += 2000;

		self._toolRotation = servoRot
		self._gripper = servoGrab

		result = self._write1444122read1(CMD_STEPS, j1, j2, j3, control, self.reverseBits16(servoGrab), self.reverseBits16(servoRot))
		self._lock.release()
		return result

	def ExecQueue(self):
		'''
		Executes deferred commands.
		'''
		raise NotImplementedError()
		self._lock.acquire()
		result = self._write0(CMD_EXEC_QUEUE)
		self._lock.release()
		return result

	def GetAccelerometers(self):
		'''
		Returns data aquired from accelerometers at power on.
		There are 17 reads in FPGA version and 20 reads in RAMPS version of each accelerometer
		that the firmware does and then averages the result before returning it here.
		'''
		self._lock.acquire()
		result = self._reads222222(CMD_GET_ACCELS)
		self._lock.release()
		return result

	def GetCounters(self):
		'''
		'''
		self._lock.acquire()
		result = self._reads444(CMD_GET_COUNTERS)
		self._lock.release()
		return result

	def SetCounters(self, base, rear, fore):
		'''
		'''
		self._lock.acquire()
		result = self._write444(CMD_SET_COUNTERS, base, rear, fore)
		self._lock.release()
		return result

	def SwitchToAccelerometerReportMode(self):
		'''
		Apparently the following won't work because of the way dobot was desgined
		and limitations of AVR - cannot switch SPI from Slave to Master back.
		So, as a workaround, just hold the "Sensor Calibration" button and start your
		app. Arduino is reset on serial port connection and it takes about 2 seconds
		for it to start. After that you can release the button. That switches dobot to
		accelerometer reporting mode. To move the arm turn off the power switch.

		This function is left just in case a proper fix comes up.

		Switches dobot to accelerometer report mode.
		Dobot must be reset to enter normal mode after issuing this command.
		'''
		raise NotImplementedError('Read function description for more info')
		self._lock.acquire()
		result = self._write_read(CMD_SWITCH_TO_ACCEL_REPORT_MODE, [])
		self._lock.release()
		return result

	def LaserOn(self, on):
		'''
		@return Returns a tuple where the first element tells whether the command has been successfully
		received (0 - yes, 1 - timed out), and the second element tells whether the command was added
		to the controller's command queue (1 - added, 0 - not added, as the queue was full).
		'''
		self._lock.acquire()
		if on:
			result = self._write1read1(CMD_LASER_ON, 1)
		else:
			result = self._write1read1(CMD_LASER_ON, 0)
		self._lock.release()
		return result
	
	def PumpOn(self, on):
		'''
		Turn On/Off the pump motor, if you want actually grip something you also need to turn the valve
		on otherwise the air just flow throught it.
		
		@return Returns a tuple where the first element tells whether the command has been successfully
		received (0 - yes, 1 - timed out), and the second element tells whether the command was added
		to the controller's command queue (1 - added, 0 - not added, as the queue was full).
		'''
		self._lock.acquire()
		if on:
			result = self._write1read1(CMD_PUMP_ON, 1)
		else:
			result = self._write1read1(CMD_PUMP_ON, 0)
		self._lock.release()
		return result
	
	def ValveOn(self, on):
		'''
		Does not do much by itself but work in tandem with the pump, when the pump is on turning the valve
		on will allow you to grab things, turn it off to release the air pressure.
		
		@return Returns a tuple where the first element tells whether the command has been successfully
		received (0 - yes, 1 - timed out), and the second element tells whether the command was added
		to the controller's command queue (1 - added, 0 - not added, as the queue was full).
		'''
		self._lock.acquire()
		if on:
			result = self._write1read1(CMD_VALVE_ON, 1)
		else:
			result = self._write1read1(CMD_VALVE_ON, 0)
		self._lock.release()
		return result

	def Wait(self, waitTime):
		'''
		Makes the arm wait in current position for the specified period of time. The wait period is specified
		in seconds and can be fractions of seconds.
		The resolution of this command is up to 20ms.

		In order to make the arm wait a number of commands is issued to do nothing. Each command takes 20ms
		to execute by the arm.
		'''
		iterations = int(waitTime * 50)
		for i in range(iterations):
			ret = (0, 0)
			# Keep sending until buffered
			while not ret[0] or not ret[1]:
				ret = self.Steps(0, 0, 0, 0, 0, 0, self._gripper, self._toolRotation)

	def BoardVersion(self):
		'''
		Checks board version.
		Returns (success, version), where version=0 is FPGA and version=1 is RAMPS.
		'''
		self._lock.acquire()
		result = self._read1(CMD_BOARD_VERSION)
		self._lock.release()
		return result

	def isFpga(self):
		return self._ramps == self.FPGA

	def isRamps(self):
		return self._ramps == self.RAMPS

	def isReady(self):
		'''
		Checks whether the controller is up and running.
		'''
		self._lock.acquire()
		result = self._read1(CMD_READY)
		self._lock.release()
		# Check for magic number.
		# return [result[0], result[1] == 0x40]
		return result

	def reset(self):
#		self._lock.acquire()
		i = 0
		while i < 5:
			self._port.read(1)
			i += 1
		self._crc_clear()
#		self._lock.release()

"""
open-dobot serial aggregator.

This is a workaround to send data in bursts on systems that have slow API
used by pyserial (e.g. Windows).

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

	def close(self):
		self._ser.close()
