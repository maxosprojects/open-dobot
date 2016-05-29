/*
open-dobot firmware.

Find driver and SDK at https://github.com/maxosprojects/open-dobot

Author: maxosprojects (March 18 2016)
Additional Authors: <put your name here>

Version: 1.2.2

License: MIT
*/

#include "../dobot.h"
#include "queue.h"

CommandQueue::CommandQueue(unsigned int newSize) {
  // queue = new Command[newSize];
  queue = (Command*) malloc(sizeof(Command) * newSize);
  head = 0;
  tail = 0;
  size = newSize;
};

byte CommandQueue::appendHead(ulong *newJ1, ulong *newJ2, ulong *newJ3, byte *control, uint servoGrab, uint servoRot, CommandType type) {
  if (!isFull()) {
    queue[head].j1 = *newJ1;
    queue[head].j2 = *newJ2;
    queue[head].j3 = *newJ3;
    queue[head].control = *control;
    queue[head].servoGrab = servoGrab;
    queue[head].servoRot = servoRot;
    queue[head].type = type;
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

Command* CommandQueue::appendHead() {
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

Command* CommandQueue::peekTail() {
  return &queue[tail];
};

Command* CommandQueue::popTail() {
  Command* ptr = &queue[tail];
  tail++;
  if (tail >= size) {
    tail = 0;
  }
  return ptr;
};

void CommandQueue::clear() {
  head = 0;
  tail = 0;
};

byte CommandQueue::isEmpty() {
  return head == tail;
};

byte CommandQueue::isFull() {
  int diff = head - tail;
  if (diff == -1 || diff == (size - 1)) {
    return 1;
  }
  return 0;
};
