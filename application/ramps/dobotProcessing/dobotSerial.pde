import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/*
open-dobot driver for RAMPS version.

Find improved FPGA version, driver and SDK at https://github.com/maxosprojects/open-dobot

Author: maxosprojects (March 18 2016)
Additional Authors: <put your name here>

License: MIT
*/

enum CMD {
  CMD_READY,
  CMD_STEPS,
  CMD_EXEC_QUEUE;
}

class IntTuple {
  public int el0;
  public int el1;
  
  IntTuple(int el0, int el1) {
    this.el0 = el0;
    this.el1 = el1;
  }
}

class Dobot {
  private int max_trys = 3;
  private Lock lock = new ReentrantLock();
  private String comport;
  private int rate;
  private Serial port;
  private int crc;
  // Timeout in ms.
  private int timeout = 100;

  protected Dobot() {
  }

  Dobot(String comport, int rate) {
    this.comport = comport;
    this.rate = rate;
    this.crc = 0xFFFF;
  }

  public void open(PApplet parent) {
    this.port = new Serial(parent, this.comport, this.rate);
  }

  public void open(PApplet parent, Integer timeout) {
    this.timeout = timeout;
    this.open(parent);
  }

  public void close() {
    this.port.stop();
  }

  private void crc_clear() {
    this.crc = 0xFFFF;
  }

  private void crc_update(int data) {
    this.crc = this.crc ^ ((data&0xFF) << 8);
    for (int i = 0; i < 8; i++) {
      if ((this.crc&0x8000) == 0x8000) {
        this.crc = ((this.crc << 1) ^ 0x1021);
      } else {
        this.crc = this.crc << 1;
      }
    }
  }
  
  private int read() {
    int start = millis();
    while (millis() - start < this.timeout) {
      int data = this.port.read();
      // Without yielding UI is blocked and Serial is not updating buffer => always error.
      Thread.yield();
      if (data > -1) {
        return data;
      }
    }
    return -1;
  }
  
  private int[] read(int n) {
    int data[] = new int[n];
    for (int i = 0; i < n; i++) {
      data[i] = this.read();
      if (data[i] == -1) {
        return new int[0];
      }
    }
    return data;
  }

  private IntTuple readchecksumword() {
    int data[] = this.read(2);
    if (data.length == 2) {
      this.crc = (data[0]<<8) | data[1];
      return new IntTuple(1, this.crc);
    }
    return new IntTuple(0, 0);
  }

  private IntTuple readbyte() {
    int data[] = this.read(1);
    if (data.length == 1) {
      int val = data[0];
      this.crc_update(val);
      return new IntTuple(1, val);
    }
    return new IntTuple(0, 0);
  }

  private IntTuple readlong() {
    IntTuple val1 = this.readbyte();
    if (val1.el0 != 0) {
      IntTuple val2 = this.readbyte();
      if (val2.el0 != 0) {
        IntTuple val3 = this.readbyte();
        if (val3.el0 != 0) {
          IntTuple val4 = this.readbyte();
          if (val4.el0 != 0) {
            return new IntTuple(1, val1.el1<<24|val2.el1<<16|val3.el1<<8|val4.el1);
          }
        }
      }
    }
    return new IntTuple(0, 0);
  }

  private IntTuple readslong() {
    IntTuple val = this.readlong();
    if (val.el0 != 0) {
      if ((val.el1&0x80000000) != 0) {
        return new IntTuple(val.el0, val.el1-Integer.MAX_VALUE);
      }
      return new IntTuple(val.el0, val.el1);
    }
    return new IntTuple(0, 0);
  }

  private IntTuple read1(CMD cmd) {
    int trys = this.max_trys;
    while (trys > 0) {
      this.port.clear();
      this.sendcommand(cmd);
      IntTuple val1 = this.readbyte();
      if (val1.el0 != 0) {
        IntTuple localCrc = this.readchecksumword();
        if (localCrc.el0 != 0) {
          if ((this.crc&0xFFFF) != (localCrc.el1&0xFFFF)) {
            throw new RuntimeException(String.format("crc differs, %d, %d", this.crc, localCrc.el1));
          }
          return new IntTuple(1, val1.el1);
        }
      }
      trys--;
    }
    //raise Exception("couldn't get response in time for", _max_trys, 'times')
    return new IntTuple(0, 0);
  }

  private IntTuple read4(CMD cmd) {
    int trys = this.max_trys;
    while (trys > 0) {
      this.port.clear();
      this.sendcommand(cmd);
      IntTuple val1 = this.readlong();
      if (val1.el0 != 0) {
        IntTuple localCrc = this.readchecksumword();
        if (localCrc.el0 != 0) {
          if ((this.crc&0xFFFF) != (localCrc.el1&0xFFFF)) {
            return new IntTuple(0, 0);
          }
          return new IntTuple(1, val1.el1);
        }
      }
      trys--;
    }
    return new IntTuple(0, 0);
  }

  //private IntTuple read4_1(CMD cmd) {
  //  int trys = this.max_trys;
  //  while (trys > 0) {
  //    this.port.clear();
  //    this.sendcommand(cmd);
  //    IntTuple val1 = this.readslong();
  //    if (val1.el0 != 0) {
  //      IntTuple val2 = this.readbyte();
  //      if (val2.el0 != 0) {
  //        IntTuple localCrc = this.readchecksumword();
  //        if (localCrc.el0 != 0) {
  //          if ((this.crc&0xFFFF) != (localCrc.el1&0xFFFF)) {
  //            return new IntTuple(0, 0);
  //          }
  //          return new IntTuple(1, val1.el1, val2.el1);
  //        }
  //      }
  //    }
  //    trys--;
  //  }
  //  return new IntTuple(0, 0);
  //}

  private void writebyte(int val) {
    this.crc_update(val&0xFF);
    this.port.write(val&0xFF);
  }

  private void writeword(int val) {
    this.writebyte((val>>8)&0xFF);
    this.writebyte(val&0xFF);
  }

  private void writelong(int val) {
    this.writebyte((val>>24)&0xFF);
    this.writebyte((val>>16)&0xFF);
    this.writebyte((val>>8)&0xFF);
    this.writebyte(val&0xFF);
  }

  private boolean writechecksum() {
    this.writeword(this.crc&0xFFFF);
    IntTuple val = this.readbyte();
    if (val.el0 != 0) {
      return true;
    }
    return false;
  }

  private void sendcommand(CMD command) {
    int cmdVal = command.ordinal();
    this.crc_clear();
    this.crc_update(cmdVal);
    port.write(cmdVal);
  }

  private boolean write0(CMD cmd) {
    int trys = this.max_trys;
    while (trys > 0) {
      this.sendcommand(cmd);
      if (this.writechecksum()) {
        return true;
      }
      trys--;
    }
    return false;
  }

  private boolean write1(CMD cmd, int val) {
    int trys = this.max_trys;
    while (trys > 0) {
      this.sendcommand(cmd);
      this.writebyte(val);
      if (this.writechecksum()) {
        return true;
      }
      trys--;
    }
    return false;
  }

  private boolean write2(CMD cmd, int val) {
    int trys = this.max_trys;
    while (trys > 0) {
      this.sendcommand(cmd);
      this.writeword(val);
      if (this.writechecksum()) {
        return true;
      }
      trys--;
    }
    return false;
  }

  private boolean write4(CMD cmd, int val) {
    int trys = this.max_trys;
    while (trys > 0) {
      this.sendcommand(cmd);
      this.writelong(val);
      if (this.writechecksum()) {
        return true;
      }
      trys--;
    }
    return false;
  }

  private boolean write14(CMD cmd, int val1, int val2) {
    int trys = this.max_trys;
    while (trys > 0) {
      this.sendcommand(cmd);
      this.writebyte(val1);
      this.writelong(val2);
      if (this.writechecksum()) {
        return true;
      }
      trys--;
    }
    return false;
  }

  private IntTuple write11121read1(CMD cmd, int val1, int val2, int val3, int val4, int val5) {
    int trys = this.max_trys;
    while (trys > 0) {
      this.sendcommand(cmd);
      this.writebyte(val1);
      this.writebyte(val2);
      this.writebyte(val3);
      this.writeword(val4);
      this.writebyte(val5);
      this.writeword(this.crc&0xFFFF);
      this.port.clear();
      this.crc_clear();
      IntTuple ret = this.readbyte();
      if (ret.el0 != 0) {
        IntTuple localCrc = this.readchecksumword();
        if (localCrc.el0 != 0) {
          if ((this.crc&0xFFFF) != (localCrc.el1&0xFFFF)) {
            throw new RuntimeException(String.format("crc differs, %d, %d", this.crc, localCrc));
          }
          return new IntTuple(1, ret.el1);
        }
      }
      trys--;
    }
    return new IntTuple(0, 0);
  }

  //Adds a command to the controller queue.
  //Controller runs a timer that counts ticks (around 3.47kHz). "Steps" command specifies
  //a scaler for each joint (joint1 - base, and so on, according to the official docs).
  //On every "ticks modulo scaler == 0" the STEP pin (on the stepper driver) is toggled.
  //So, if scaler=1, then the STEP pin is toggled at around 3.47kHz frequency. If scaler
  //is 10, then STEP pin is toggled at around 370.
  
  //The "Steps" command also specifies the number of ticks to run the command for, the
  //state for DIR pin (on the stepper driver) to indicate which direction the motor will
  //turn, and the deferred flag to indicate whether the command should not be executed until
  //the "ExecQueue" command is issued (for precise and smooth trajectories built of many
  //short-running commands).

  //@param j1 - joint1 scaler
  //@param j2 - joint2 scaler
  //@param j3 - joint3 scaler
  //@param ticks - number of ticks to run the command for
  //@param j1dir - direction for joint1
  //@param j2dir - direction for joint2
  //@param j3dir - direction for joint3
  //@param deferred - defer execution of this command and all commands issued after this until
  //        the "ExecQueue" command is issued.
  //@return Returns a tuple where the first element tells whether the command has been successfully
  //received (0 - yes, 1 - timed out), and the second element tells whether the command was added
  //to the controller's command queue (1 - added, 0 - not added, as the queue was full).
  public IntTuple steps(int j1, int j2, int j3, int ticks, int j1dir, int j2dir, int j3dir, boolean deferred) {
    int control = ((j1dir & 0x01) << 1) | ((j2dir & 0x01) << 2) | ((j3dir & 0x01) << 3);
    if (deferred) {
      control |= 0x01;
    }
    this.lock.lock();
    IntTuple result = this.write11121read1(CMD.CMD_STEPS, j1, j2, j3, ticks, control);
    this.lock.unlock();
    return result;
  }

  //Executes deferred commands.
  public boolean ExecQueue() {
    this.lock.lock();
    boolean result = this.write0(CMD.CMD_EXEC_QUEUE);
    this.lock.unlock();
    return result;
  }

  //Checks whether the controller is up and running.
  public IntTuple isReady() {
    this.lock.lock();
    IntTuple result = this.read1(CMD.CMD_READY);
    this.lock.unlock();
    //Check for magic number.
    //return [result[0], result[1] == 0x40]
    return result;
  }

  public void reset() {
    //self._lock.acquire()
    int i = 0;
    while (i < 5) {
      this.port.clear();
      this.port.read();
      i++;
    }
    this.crc_clear();
    //self._lock.release()
  }
}


class FakeDobot extends Dobot {

  FakeDobot(String comport, int rate) {
  }

  public void open(PApplet parent) {
  }

  public void open(PApplet parent, Integer timeout) {
  }

  public void close() {
  }

  public IntTuple steps(int j1, int j2, int j3, int ticks, int j1dir, int j2dir, int j3dir, boolean deferred) {
    return new IntTuple(1, 1);
  }

  public boolean ExecQueue() {
    return true;
  }

  public IntTuple isReady() {
    return new IntTuple(1, 0x40);
  }

  public void reset() {
  }
}


//void dobotSerialBegin()
//{
//    String portName = "/dev/tty.usbmodem1421";
//    myPort = new Serial(this, portName, 115200);
//    myPort.bufferUntil(0x5a);
//    serialEn = true;
//}

//void sendDeltaXYZ(float deltaX, float deltaY, float deltaZ, float StartVel, float EndVel, float MaxVel)
//{
//  float state = 4.0;
//  float Axis = 0;
//  float X = deltaX;
//  float Y = deltaY;
//  float Z = deltaZ;
//  float RHead = 0;
//  float isGrab = 0;
//  //StartVel = StartVel;
//  //EndVel = EndVel;
//  //MaxVel = MaxVel;

//  sendPackage(state, Axis, X, Y, Z, RHead, isGrab, StartVel, EndVel, MaxVel);  
//  println("state:" + state + " axis:" + Axis + " X:" + X + " Y:" + Y + " Z:" + Z + " RHead:" + RHead + " isGrab:" + isGrab + " StarVel:"+ StartVel + " EndVel:" + EndVel + " MaxVel:" + MaxVel);
//}

//void sendXYZ(float X, float Y, float Z)
//{
//  float cmd = 3.0;
//  float RHead = 0;
//  float isGrab = 0;

//  sendPackage(cmd, 0, X, Y, Z, RHead, isGrab, 1, 0, 0);
////  println("state:" + state + " axis:" + Axis + " X:" + X + " Y:" + Y + " Z:" + Z + " RHead:" + RHead + " isGrab:" + isGrab + " StarVel:"+ StartVel + " EndVel:" + EndVel + " MaxVel:" + MaxVel);
//}


//void sendPackage(float state, float Axis, float X, float Y, float Z, float RHead, float isGrab, float StartVel, float EndVel, float MaxVel)
//{
//  byte[][] send = new byte[10][4];

//  send[0] = float2byte(state);
//  send[1] = float2byte(Axis);
//  send[2] = float2byte(X);
//  send[3] = float2byte(Y);
//  send[4] = float2byte(Z);
//  send[5] = float2byte(RHead);
//  send[6] = float2byte(isGrab);
//  send[7] = float2byte(StartVel);
//  send[8] = float2byte(EndVel);
//  send[9] = float2byte(MaxVel);  

//  myPort.write(byte(0xa5)); //the package head
//  for (int i =0; i<10; i++) {
//    myPort.write(send[i]);
//  }
//  myPort.write(byte(0x5a));  //the package tail
//}

//void sendBeginPackage()
//{
//  byte[] send= {
//    byte(0xa5), byte(0x00), byte(0x00), byte(0x11), byte(0x11), 
//    byte(0x22), byte(0x22), byte(0x33), byte(0x33), byte(0x00), 
//    byte(0x00), byte(0x00), byte(0x00), byte(0x00), byte(0x00), 
//    byte(0x00), byte(0x00), byte(0x00), byte(0x00), byte(0x00), 
//    byte(0x00), byte(0x00), byte(0x00), byte(0x00), byte(0x00), 
//    byte(0x00), byte(0x00), byte(0x00), byte(0x00), byte(0x00), 
//    byte(0x00), byte(0x00), byte(0x00), byte(0x00), byte(0x00), 
//    byte(0x00), byte(0x00), byte(0x00), byte(0x00), byte(0x00), 
//    byte(0x00), byte(0x5a)
//    };
//    myPort.write(send);
////    decodeBeginPackage(send);
//}
////float[] receivedPackage()
////{
////  byte[] inBuffer = new byte[4];
////  float[] packages = new float[9];
////  while (myPort.available() > 0) 
////  {
////    if (inBuffer != null)
////    {
////      for(int i=0;i<9;i++)
////      {
////        inBuffer = myPort.readBytes();
////        packages[i] = byte2float(inBuffer, 4);
//// //       println(packages[i]);
////      }
////    }
////  }
////  for(int i=0; i<9; i++)
////  println(packages[i]);

////  return packages;

////}

//byte[] float2byte(float f) 
//{  
//  int fbit = Float.floatToIntBits(f);  

//  byte[] b = new byte[4];    
//  for (int i = 0; i < 4; i++)
//  {    
//    b[i] = (byte) (fbit >> (24 - i * 8));
//  }

//  int len = b.length;  
//  byte[] dest = new byte[len];  
//  System.arraycopy(b, 0, dest, 0, len);  
//  byte temp;
//  for (int i = 0; i < len / 2; ++i)
//  {  
//    temp = dest[i];  
//    dest[i] = dest[len - i - 1];  
//    dest[len - i - 1] = temp;
//  }
//  return dest;
//}

//float byte2float(byte[] b, int index) 
//{    
//  int l;                                             
//  l = b[index + 0];                                  
//  l &= 0xff;                                         
//  l |= ((long) b[index + 1] << 8);                   
//  l &= 0xffff;                                       
//  l |= ((long) b[index + 2] << 16);                  
//  l &= 0xffffff;                                     
//  l |= ((long) b[index + 3] << 24);                  
//  return Float.intBitsToFloat(l);
//}

//void decodeBeginPackage(byte[] b) {
//  println("Begin Package:");
//  for (int i = 1; i < b.length - 1; i++) {
//    if ((i - 1) % 4 == 0) {
//      println(byte2float(b, i));
//    }
//  }
//}