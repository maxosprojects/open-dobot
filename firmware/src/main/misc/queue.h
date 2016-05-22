
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
