/*
open-dobot firmware.

Find driver and SDK at https://github.com/maxosprojects/open-dobot

Author: maxosprojects (March 18 2016)
Additional Authors: <put your name here>

Version: 1.2.2

License: MIT
*/

// #define DEBUG ENABLED
#define DEBUG_DDR DDRH
#define DEBUG_PORT PORTH
// Fpga
// #define DEBUG_PIN PORTH5
// Ramps
#define DEBUG_PIN PORTH1

extern byte accelReportMode;
extern int cmdPtrArrayLastIndex;
extern funcPtrs cmdArray[];

inline byte processCommand() {
  if (cmdInBuffIndex > 0) {
    if ((accelReportMode
              && cmd[0] != CMD_GET_ACCELS
              && cmd[0] != CMD_BOARD_VERSION)
          || cmd[0] > cmdPtrArrayLastIndex) {
      cmdInBuffIndex = 0;
      return 0;
    }
    return cmdArray[cmd[0]]();
  }
  return 0;
}

inline void initDebug() {
#ifdef DEBUG
  DEBUG_DDR |= (1<<DEBUG_PIN);
#endif
}

inline void debugOn() {
#ifdef DEBUG
  DEBUG_PORT |= (1<<DEBUG_PIN);
#endif
}

inline void debugOff() {
#ifdef DEBUG
  DEBUG_PORT &= ~(1<<DEBUG_PIN);
#endif
}

inline void serialWrite(byte c) {
  if (BLUETOOTH_ENABLED) {
    loop_until_bit_is_set(UCSR1A, UDRE1);
    UDR1 = c;
  } else {
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
  }
}

inline void serialWrite(byte data[], byte num) {
  if (BLUETOOTH_ENABLED) {
    for (byte i = 0; i < num; i++) {
      loop_until_bit_is_set(UCSR1A, UDRE1);
      UDR1 = data[i];
    }
  } else {
    for (byte i = 0; i < num; i++) {
      loop_until_bit_is_set(UCSR0A, UDRE0);
      UDR0 = data[i];
    }
  }
}

// Returns number of bytes read or 0 if timeout occurred.
// Timeout: 1000 increments ~ 600us, so allow about 9ms interbyte and 18ms overall.
inline byte serialReadNum(byte data[], byte num) {
  unsigned int interByteTimeout = 0;
  unsigned int transactionTimeout = 0;
  byte cnt = 0;
  byte tmp;
  if (BLUETOOTH_ENABLED) {
    while (cnt < num) {
      // Wait until data exists.
      while (!(UCSR1A & (1<<RXC1))) {
        interByteTimeout++;
        transactionTimeout++;
        if (interByteTimeout > 15000 || transactionTimeout > 30000) {
          return 0;
        }
      }
      interByteTimeout = 0;
      tmp = UDR1;
      data[cnt++] = tmp;
    }
  } else {
    while (cnt < num) {
      // Wait until data exists.
      while (!(UCSR0A & (1<<RXC0))) {
        interByteTimeout++;
        transactionTimeout++;
        if (interByteTimeout > 15000 || transactionTimeout > 30000) {
          return 0;
        }
      }
      interByteTimeout = 0;
      tmp = UDR0;
      data[cnt++] = tmp;
    }
  }
  return cnt;
}

inline void serialRead() {
  // This is done this way for code speed optimization purposes.
  // Don't change, otherwise SPI bytes are skipped and commands lost.
  if (BLUETOOTH_ENABLED) {
    if (UCSR1A & (1<<RXC1)) {
      cmd[cmdInBuffIndex] = UDR1;
      if (cmdInBuffIndex < 23) {
        cmdInBuffIndex++;
      }
    }
  } else {
    if (UCSR0A & (1<<RXC0)) {
      cmd[cmdInBuffIndex] = UDR0;
      if (cmdInBuffIndex < 23) {
        cmdInBuffIndex++;
      }
    }
  }
}
