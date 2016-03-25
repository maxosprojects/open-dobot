
#define F_CPU 16000000UL
#define BAUD 115200
#include <util/setbaud.h>

#include "dobot.h"

#define SPI_PORT PORTB
#define SPI_DDR  DDRB
#define SPI_MISO PORTB3

#define FPGA_ENABLE_PORT PORTG
#define FPGA_ENABLE_DDR  DDRG
#define FPGA_ENABLE_PIN PORTG1
#define FPGA_COMMAND_PORT PORTL
#define FPGA_COMMAND_DDR DDRL
#define FPGA_COMMAND_PIN PORTL7
#define FPGA_COMMAND_ACCELS_PIN PORTL0

#define CMD_QUEUE_SIZE     200

CommandQueue cmdQueue(CMD_QUEUE_SIZE);

#define CMD_READY 0
#define CMD_STEPS 1
#define CMD_EXEC_QUEUE 2
// DO NOT FORGET TO UPDATE cmdArray SIZE!
funcPtrs cmdArray[3];
// Last index in the commands array.
int cmdLastIndex;

byte cmd[20];
byte crc[2];

// ulong lastTimeExecuted = 0;
byte defer = 1;

void serialInit(void) {
  UBRR0H = UBRRH_VALUE;
  UBRR0L = UBRRL_VALUE;

#if USE_2X
  UCSR0A |= _BV(U2X0);
#else
  UCSR0A &= ~(_BV(U2X0));
#endif

  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
  UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */
}

void serialWrite(byte c) {
  UDR0 = c;
  loop_until_bit_is_set(UCSR0A, TXC0); /* Wait until transmission ready. */
}

void serialWrite(byte data[], byte num) {
  for (byte i = 0; i < num; i++) {
    UDR0 = data[i];
    loop_until_bit_is_set(UCSR0A, TXC0); /* Wait until transmission ready. */
  }
}

// Returns number of bytes read.
byte serialReadNum(byte data[], byte num) {
  unsigned int i = 0;
  byte cnt = 0;
  while (cnt < num) {
    // Wait until data exists.
    while (!(UCSR0A & (1<<RXC0))) {
      if (i > 1000) {
        return 0;
      }
    }
    i = 0;
    data[cnt++] = UDR0;
  }
  return cnt;
}

void setup() {
  cmdArray[CMD_READY] = cmdReady;
  cmdArray[CMD_STEPS] = cmdSteps;
  cmdArray[CMD_EXEC_QUEUE] = cmdExecQueue;
  cmdLastIndex = sizeof(cmdArray) / sizeof(cmdArray[0]) - 1;

  // have to send on master in, *slave out*
  SPI_DDR = (1<<SPI_MISO);

  FPGA_ENABLE_DDR = (1<<FPGA_ENABLE_PIN);
  FPGA_COMMAND_DDR = (1<<FPGA_COMMAND_PIN) | (1<<FPGA_COMMAND_ACCELS_PIN);

  FPGA_COMMAND_PORT &= (0<<FPGA_COMMAND_PIN);
  FPGA_COMMAND_PORT &= (0<<FPGA_COMMAND_ACCELS_PIN);

  // turn on SPI in slave mode
  // CPOL=0, CPHA=1 - Trailing (Falling) Edge
  // enable interrupt
  //  SPCR = _BV(SPE) | _BV(CPHA) | _BV(SPIE);
  SPCR = _BV(SPE) | _BV(CPHA);

  // FPGA_ENABLE_PORT |= (1<<FPGA_ENABLE_PIN);

  // Serial.begin(115200);
  // while (!Serial) {
  //   ; // wait for serial port to connect.
  // }
  // // Set timeout for readBytes() to 100 ms.
  // Serial.setTimeout(100);
  serialInit();
}

void processCommand() {
  // If something is waiting in the serial port to be read...
  if (UCSR0A&(1<<RXC0)) {
    // Get incoming byte.
    cmd[0] = UDR0;
    if (cmd[0] > cmdLastIndex) {
      return;
    }
    cmdArray[cmd[0]]();
  }
}

// CMD: returns magic number to indicate that the controller is alive.
void cmdReady() {
  crcCcitt(cmd, 1);
  // Return magic number.
  cmd[0] = 0x40;
  write1(cmd);
}

// CMD: adds a command to the queue.
void cmdSteps() {
  serialWrite(1);
  if (!read13(&cmd[1])) {
    return;
  }
  if (checkCrc(cmd, 14)) {
    resetCrc();
    if (cmdQueue.appendHead((ulong*) &cmd[1], (ulong*) &cmd[2], (ulong*) &cmd[3], cmd[6])) {
      cmd[0] = 1;
      write1(cmd);
    } else {
      cmd[0] = 0;
      write1(cmd);
    }
  }
}

// CMD: Execute deferred commands in the queue.
void cmdExecQueue() {
  defer = 0;
}

byte read2(byte data[]) {
  return serialReadNum(data, 2);
}

byte read13(byte data[]) {
  return serialReadNum(data, 13);
}

void write1(byte data[]) {
  crcCcitt(data, 1, 1);
  data[1] = crc[0];
  data[2] = crc[1];
  serialWrite(data, 3);
}

int dataToInt(byte data[]) {
  int result = ((((uint16_t) data[0]) << 8) & 0xFF00) | (((uint16_t) data[1]) & 0x00FF);
  return result;
}

uint16_t dataToUint(byte data[]) {
  return (uint16_t) dataToInt(data);
}

byte checkCrc(byte data[], int len) {
  if (!read2(&cmd[len])) {
    return 0;
  }
  crcCcitt(cmd, len);
  if (data[len] == crc[0] && data[len + 1] == crc[1]) {
    return 1;
  }
  return 0;
}

byte confirmCrc(byte data[], int len) {
  if (checkCrc(data, len)) {
    serialWrite(1);
    return 1;
  }
  return 0;
}

void resetCrc() {
  crc[0] = 0xff;
  crc[1] = 0xff;
}

void crcCcitt(byte data[], int len) {
  crcCcitt(data, len, false);
}

void crcCcitt(byte data[], int len, byte keepSeed) {
  uint16_t localCrc;
  if (keepSeed) {
    localCrc = ((((uint16_t) crc[0]) << 8) & 0xFF00) | (((uint16_t) crc[1]) & 0x00FF);
  } else {
    localCrc = 0xffff;
  }
  for (int i = 0; i < len; i++) {
    localCrc = localCrc ^ ( (((uint16_t) data[i]) << 8) & 0xff00);
    for (int n = 0; n < 8; n++) {
      if ((localCrc & 0x8000) == 0x8000) {
        localCrc = ((localCrc << 1) ^ 0x1021);
      } else {
        localCrc = localCrc << 1;
      }
    }
  }
  crc[0] = (byte) (localCrc >> 8);
  crc[1] = (byte) localCrc;
}

inline void writeSpiByte(byte data) {
  byte junk;

  SPDR = data;
  loop_until_bit_is_set(SPSR, SPIF);
  junk = SPDR;
}

void writeSpi(Command* cmd) {
  byte* data = (byte*)cmd;

  FPGA_COMMAND_PORT |= (1<<FPGA_COMMAND_PIN);
  loop_until_bit_is_set(SPSR, SPIF);

  writeSpiByte(sequenceRest[0]);
  for (byte i = 0; i < 13; i++) {
    writeSpiByte(data[i]);
  }
  writeSpiByte(sequenceRest[14]);
  writeSpiByte(sequenceRest[15]);
  writeSpiByte(sequenceRest[16]);
  writeSpiByte(sequenceRest[17]);
  writeSpiByte(sequenceRest[18]);
  FPGA_COMMAND_PORT &= (0<<FPGA_COMMAND_PIN);
}

int main() {
  setup();
  FPGA_ENABLE_PORT |= (1<<FPGA_ENABLE_PIN);

  byte i = 0;
  byte data;
  while (1) {
    if (SPDR == 0x5a) {
      SPDR = 0x00;
      if (cmdQueue.isEmpty()) {
        writeSpi((Command*) &sequenceRest[1]);
      } else {
        writeSpi(cmdQueue.popTail());
      }
      processCommand();
    }
  }
  return 0;
}
