import struct

'''
frequency: original number
where frequency is not used and original number has been decoded.
There are 50 commands/second dobot executes (50Hz). So if 50 commands are sent dobot will execute for one second.

--== Decoding ==--
Those original numbers can be decoded as follows:
1. Take the number and strip the last byte
2. Reverse bits (so that bit 0 becomes bit 23, bit 1 becomes bit 22 and so on)
3. Divide 25+e6 (25mil) over the result of item 2. This gives the frequency

--== Encoding ==--
The reverse procedure, to make steppers step at desired frequency:
1. 25+e6 divide by desired frequency
2. Reverse bits

--== Converting to/from number of motor steps ==--
From steps:
1. 5+e5 divide by the desired number of steps
2. Reverse bits

'''
freq = {
	   0: 0x0242f000,
	  50: 0xf885e000,
	 250: 0xf9618000,
	 400: 0xc42f0000,
	 600: 0x834500d5,
	 800: 0x885e0000,
	 950: 0x53660053,
	1150: 0x572a0084,
	1300: 0xb8d200a3,
	1450: 0x1ac20086
}

results = {
	 5: 0x90,
	12: 0xe8,
	18: 0xc4,
	25: 0x8c,
	30: 0xdc
}

def longToBytes(val):
	return (ord(c) for c in struct.pack('<L', val))

def bytesToFloat(bytes):
	data = ''.join(chr(i) for i in bytes)
	return struct.unpack('<f', data)[0]

def longToFloat(val):
	return bytesToFloat(longToBytes(val))

def getByte(val, index):
	return (val >> (index * 8)) & 0xFF

def getWord(val, index):
	return (val >> (index * 16)) & 0xFFFF

def takeNLeftBytes(val, n):
	return val >> ((4 - n) * 8)

def takeNRightBytes(val, n):
	return val & (0xFFFFFFFF >> ((4 - n) * 8))

def reverseBits32(val):
	rev = 0
	for i in range(0, 32):
		rev |= (((val & 0xFFFFFFFF) >> i) & 0x00000001) << (31 - i)
	return rev & 0xFFFFFFFF

def reverseBits8(val):
	rev = 0
	for i in range(0, 8):
		rev |= (((val & 0xFF) >> i) & 0x01) << (7 - i)
	return rev & 0xFF

for key in sorted(freq.keys()):
	val = freq[key]
	print '{0:4d} {1:10d} 0x{1:08X} {1:032b} {7:6d} {6:6d} {7:016b} {6:016b} {2:3d} {3:3d} {4:3d} {5:3d} {2:08b} {3:08b} {4:08b} {5:08b}'.format(key, val, getByte(val, 3), getByte(val, 2), getByte(val, 1), getByte(val, 0), getWord(val, 0), getWord(val, 1))

for key in sorted(freq.keys()):
	val = freq[key]
	print '{0:4d} {1:10d} 0x{1:08X} {2:032b} {3:024b} {3:8d}'.format(key, val, reverseBits32(val), takeNRightBytes(reverseBits32(val), 3))

for key in sorted(results.keys()):
	val = results[key]
	print '{0:2d} {1:3d} 0x{1:02X} {1:08b} {2:08b} {2:3d}'.format(key, val, (reverseBits8(val) >> 1) + 1)
