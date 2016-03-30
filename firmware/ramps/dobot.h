/*
open-dobot firmware for RAMPS version.

Find improved FPGA version, driver and SDK at https://github.com/maxosprojects/open-dobot

Author: maxosprojects (March 18 2016)
Additional Authors: <put your name here>

License: MIT
*/

#ifndef DOBOT_H_INCLUDED
#define DOBOT_H_INCLUDED

typedef void(*funcPtrs)(void);

typedef struct {
  unsigned char j1dir : 1;
  unsigned char j2dir : 1;
  unsigned char j3dir : 1;
  unsigned char deferred : 1;
  unsigned char currJ1 : 1;
  unsigned char currJ2 : 1;
  unsigned char currJ3 : 1;
} ControlByte;

typedef struct {
  unsigned char j1;
  unsigned char j2;
  unsigned char j3;
  unsigned int ticks;
  ControlByte control;
} Command;

class CommandQueue {
  public:
    CommandQueue(unsigned int newSize) {
      queue = new Command[newSize];
      head = 0;
      tail = 0;
      size = newSize;
    };

    boolean appendHead(unsigned char newJ1, unsigned char newJ2, unsigned char newJ3, 
        unsigned int newTicks, unsigned char control) volatile {
      if (!isFull()) {
        processing = 1;
        queue[head].j1 = newJ1;
        queue[head].j2 = newJ2;
        queue[head].j3 = newJ3;
        queue[head].ticks = newTicks;
        queue[head].control.j1dir = (control >> 1) & 0x01;
        queue[head].control.j2dir = (control >> 2) & 0x01;
        queue[head].control.j3dir = (control >> 3) & 0x01;
        queue[head].control.deferred = control & 0x01;
        queue[head].control.currJ1 = 0;
        queue[head].control.currJ2 = 0;
        queue[head].control.currJ3 = 0;
        head++;
        if (head >= size) {
          head = 0;
        }
        processing = 0;
        return true;
      }
      return false;
    };

    volatile Command* peekTailIsr() volatile {
      return &queue[tail];
    };

    volatile Command* popTailIsr() volatile {
      volatile Command* ptr = &queue[tail];
      tail++;
      if (tail >= size) {
        tail = 0;
      }
      return ptr;
    };

    boolean isEmptyIsr() volatile {
      return head == tail;
    };

    boolean isFull() volatile {
      processing = 1;
      int diff = head - tail;
      if (diff == -1 || diff == (size - 1)) {
        processing = 0;
        return true;
      }
      processing = 0;
      return false;
    };

    unsigned char isProcessing() volatile {
      return processing;
    }

  private:
    volatile unsigned int size;
    volatile unsigned int head;
    volatile unsigned int tail;
    volatile unsigned char processing;
    volatile Command *queue;
};

#endif // DOBOT_H_INCLUDED
