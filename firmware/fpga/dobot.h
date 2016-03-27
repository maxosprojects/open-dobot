#ifndef DOBOT_H_INCLUDED
#define DOBOT_H_INCLUDED

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#define NOP __asm__ __volatile__ ("nop\n\t")

typedef unsigned char byte;
typedef unsigned int uint;
typedef unsigned long ulong;

typedef void(*funcPtrs)(void);

// typedef struct {
//   byte j1dir : 1;
//   byte j2dir : 1;
//   byte j3dir : 1;
//   // byte deferred : 1;
// } ControlByte;

typedef struct {
  ulong j1;
  ulong j2;
  ulong j3;
  // ControlByte control;
  byte control;
} Command;

void cmdReady();
void cmdSteps();
void cmdExecQueue();
void cmdGetAccels();
void cmdSwitchToAccelReportMode();
void crcCcitt(byte data[], int len);
void crcCcitt(byte data[], int len, byte keepSeed);
byte read13(byte data[]);
void write1(byte data[]);
byte write22(byte data[], uint* val1, uint* val2);
byte write4(byte data[]);
byte checkCrc(byte data[], int len);
byte confirmCrc(byte data[], int len);
void resetCrc();
void serialInit(void);
void serialWrite(byte c);
void serialWrite(byte data[], byte num);
byte serialReadNum(byte data[], byte num);
uint accelRead(unsigned char pin);

// Rest
volatile byte sequenceRest[19] = {
0xa5, // start signature
0x2,  // motor1 block start (middle driver - Stepper_Rot)
0x42,
0xf0,
0x0,
0x2,  // motor2 block start (left driver - Stepper_R)
0x42,
0xf0,
0x0,
0x2,  // motor3 block start (right driver - Stepper_L)
0x42,
0xf0,
0x0,
0xe0, // motors direction ([bit7-bit0]: bit7 - motor1, bit6 - motor2, bit5 - motor3)
0x2f,
0x80,
0x2f,
0x80,
0x5a // end signature
};

class CommandQueue {
  public:
    CommandQueue(unsigned int newSize) {
      // queue = new Command[newSize];
      queue = (Command*) malloc(sizeof(Command) * newSize);
      head = 0;
      tail = 0;
      size = newSize;
    };

    byte appendHead(ulong *newJ1, ulong *newJ2, ulong *newJ3, byte *control) {
      if (!isFull()) {
        queue[head].j1 = *newJ1;
        queue[head].j2 = *newJ2;
        queue[head].j3 = *newJ3;
        queue[head].control = *control;
        // queue[head].control.j1dir = (control >> 1) & 0x01;
        // queue[head].control.j2dir = (control >> 2) & 0x01;
        // queue[head].control.j3dir = (control >> 3) & 0x01;
        head++;
        if (head >= size) {
          head = 0;
        }
        return 1;
      }
      return 0;
    };

    Command* appendHead() {
      if (isFull()) {
        return 0;
      }
      Command* nextPlace = &queue[head];
      head++;
      if (head >= size) {
        head = 0;
      }
      return nextPlace;
    };

    Command* peekTail() {
      return &queue[tail];
    };

    Command* popTail() {
      Command* ptr = &queue[tail];
      tail++;
      if (tail >= size) {
        tail = 0;
      }
      return ptr;
    };

    byte isEmpty() {
      return head == tail;
    };

    byte isFull() {
      int diff = head - tail;
      if (diff == -1 || diff == (size - 1)) {
        return 1;
      }
      return 0;
    };

  private:
    unsigned int size;
    unsigned int head;
    unsigned int tail;
    Command *queue;
};

#endif // DOBOT_H_INCLUDED
