/*
open-dobot firmware.

Find driver and SDK at https://github.com/maxosprojects/open-dobot

Author: maxosprojects (March 18 2016)
Additional Authors: <put your name here>

Version: 1.2.2

License: MIT
*/

class CommandQueue {
  public:
    CommandQueue(unsigned int newSize);

    byte appendHead(ulong *newJ1, ulong *newJ2, ulong *newJ3, byte *control, uint servoGrab, uint servoRot, CommandType type);
    Command* appendHead();
    Command* peekTail();
    Command* popTail();
    void clear();

    byte isEmpty();
    byte isFull();

  private:
    unsigned int size;
    unsigned int head;
    unsigned int tail;
    Command *queue;
};
