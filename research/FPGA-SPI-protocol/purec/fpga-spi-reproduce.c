
// !!!!!!!!!!!
//
// HAVE TO DISCONNECT ACCELEROMETERS BEFORE FLASHING (FOR NOW) !!!
//
// !!!!!!!!!!!

#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>

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

// 50Hz on motor1 only. Response: 0x80 (1 step)
volatile unsigned char seqMove3[19] = {
0xa5, // start signature
0xf8, // motor1 block start
0x85,
0xe0,
0x0,
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

// 250Hz on motor1 only. Response: 0x90 (5 steps)
volatile unsigned char seqMove4[19] = {
0xa5, // start signature
0xf9, // motor1 block start
0x61,
0x80,
0x0,
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

// 400Hz on motor1 only. Response: 0xf0 (8 steps)
volatile unsigned char seqMove5[19] = {
0xa5, // start signature
0xc4, // motor1 block start
0x2f,
0x0,
0x0,
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

// 600Hz on motor1 only. Response: 0xe8 (12 steps)
volatile unsigned char seqMove6[19] = {
0xa5, // start signature
0x83, // motor1 block start
0x45,
0x0,
0xd5,
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

// 600Hz on motor1 only. Response: 0xe8 (12 steps)
volatile unsigned char seqMove7[19] = {
0xa5, // start signature
0x88, // motor1 block start
0x5e,
0x0,
0xd5,
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

  FPGA_ENABLE_PORT |= (1<<FPGA_ENABLE_PIN);
}

int main() {
  setup();
  unsigned char i = 0;
  unsigned char data;
  while (1) {
    if (SPDR == 0x5a) {
      SPDR = 0x00;
      FPGA_COMMAND_PORT |= (1<<FPGA_COMMAND_PIN);
      while(!(SPSR & (1<<SPIF) ));
      for (i = 0; i < 19; i++) {
        SPDR = seqMove7[i];
        while(!(SPSR & (1<<SPIF) ));
        data = SPDR;
      }
      FPGA_COMMAND_PORT &= (0<<FPGA_COMMAND_PIN);
    }
  }
  return 0;
}
