
// !!!!!!!!!!!
//
// HAVE TO DISCONNECT ACCELEROMETERS BEFORE FLASHING (FOR NOW) !!!
//
// !!!!!!!!!!!

#include <SPI.h>

const int fpgaEnablePin = 40;
const int fpgaCommandPin = 42;
const int boardAccelsPin = 49;

// Rest
volatile unsigned char seqRest[19] = {
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

// 1.45kHz on motor1 only
volatile unsigned char seqMove[19] = {
0xa5,
0x1a,
0xc2,
0x0,
0x86,
0x2,
0x42,
0xf0,
0x0,
0x2,
0x42,
0xf0,
0x0,
0xa0,
0x2f,
0x80,
0x2f,
0x80,
0x5a
};

// 1.3kHz on motor1 only
volatile unsigned char seqMove2[19] = {
0xa5, // start signature
0xb8, // motor1 block start
0xd2,
0x0,
0xa3,
0x2,  // motor2 block start
0x42,
0xf0,
0x0,
0x2,  // motor3 block start
0x42,
0xf0,
0x0,
0xe0, // direction
0x2f,
0x80,
0x2f,
0x80,
0x5a // end signature
};

void setup (void) {
  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
  pinMode(SS, INPUT);

  pinMode(fpgaEnablePin, OUTPUT);
  pinMode(fpgaCommandPin, OUTPUT);
  pinMode(boardAccelsPin, OUTPUT);

  digitalWrite(fpgaCommandPin, LOW);
  digitalWrite(boardAccelsPin, LOW);

  // turn on SPI in slave mode
  // CPOL=0, CPHA=1 - Trailing (Falling) Edge
  // enable interrupt
  //  SPCR = _BV(SPE) | _BV(CPHA) | _BV(SPIE);
  SPCR = _BV(SPE) | _BV(CPHA);

  digitalWrite(fpgaEnablePin, HIGH);
}

void loop(void) {
  unsigned char i = 0;
  byte data;
  while (true) {
    if (SPDR == 0x5a) {
      SPDR = 0x00;
      digitalWrite(fpgaCommandPin, HIGH);
      while(!(SPSR & (1<<SPIF) ));
      for (unsigned char i = 0; i < 19; i++) {
        SPDR = seqMove2[i];
        while(!(SPSR & (1<<SPIF) ));
        data = SPDR;
      }
      digitalWrite(fpgaCommandPin, LOW);
    }
  }
}
