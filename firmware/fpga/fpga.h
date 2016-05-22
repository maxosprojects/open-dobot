
/**
FPGA version routines.
**/

// CMD: Returns 0 for FPGA.
byte cmdBoardVersion() {
  // Check if not enough bytes yet.
  if (cmdInBuffIndex < 3) {
    return 1;
  }
  cmdInBuffIndex = 0;
  if (!checkCrc(cmd, 1)) {
    return 2;
  }
  // Return 0 for FPGA.
  cmd[0] = 0;
  write1(cmd);
  return 3;
}

void setupBoard() {
  //---=== Power-on sequence ===---
  // 1. FPGA_ENABLE_PIN = LOW, FPGA_COMMAND_PIN = LOW
  // 2. Check if FPGA_POWERON_PIN == HIGH. If not, then switch to accelerometer reporting mode.
  // 3. Arduino initializes - delay 900ms. Don't need to do anything.
  // 4. FPGA_COMMAND_ACCELS_INIT_PIN = HIGH
  // 5. Delay 35us
  // 6. Set SPI as Master
  // 7. Delay 35us
  // 8. Read accelerometers
  // 9. FPGA_COMMAND_ACCELS_INIT_PIN = LOW
  // 10. Delay 200ms
  // 11. Set SPI as Slave
  // 12. Enable FPGA: FPGA_ENABLE_PIN = HIGH

  //---=== Accelerometer reading sequence ===---
  // 1. Delay 800ns
  // 2. Loop 17 times:
  // 3.1. Delay 770us for accelerometer to prepare data
  // 3.2. FPGA_COMMAND_ACCEL_REAR_SS_PIN = LOW
  // 3.3. Send "RDAX / 00010000 / Read X-channel acceleration through SPI" command = 0x10
  // 3.4. Read two bytes
  // 3.5. FPGA_COMMAND_ACCEL_REAR_SS_PIN = LOW
  // 4. Average the result
  // 5. Repeat 1-3 for another accelerometer - FPGA_COMMAND_ACCEL_FRONT_SS_PIN
  
  // Power-on step 1
  FPGA_ENABLE_DDR = (1<<FPGA_ENABLE_PIN);
  FPGA_COMMAND_DDR = (1<<FPGA_COMMAND_PIN) | (1<<FPGA_COMMAND_ACCELS_INIT_PIN)
                  | (1<<FPGA_COMMAND_ACCEL_REAR_SS_PIN) | (1<<FPGA_COMMAND_ACCEL_FRONT_SS_PIN);
  FPGA_COMMAND_PORT = (1<<FPGA_COMMAND_ACCEL_REAR_SS_PIN) | (1<<FPGA_COMMAND_ACCEL_FRONT_SS_PIN);

  // Power-on step 2
  if (! (POWERON_PORT & (1<<POWERON_PIN))) {
    cmdSwitchToAccelReportMode();
  }

  // Power-on step 3
  // Do nothing.

  // Power-on step 4
  FPGA_COMMAND_PORT |= (1<<FPGA_COMMAND_ACCELS_INIT_PIN);

  // Power-on step 5
  _delay_us(35);

  // Power-on step 6
  // Set MOSI, SCK and SS as output, all others input. SS must be output for SPI remain Master. See docs.
  SPI_DDR = (1<<SPI_MOSI)|(1<<SPI_SCK)|(1<<SPI_SS);
  // Enable SPI, Master, set clock rate fck/16
  SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);

  // Power-on step 7
  _delay_us(35);

  // Power-on step 8
  accelRear = accelRead(FPGA_COMMAND_ACCEL_REAR_SS_PIN);
  accelFront = accelRead(FPGA_COMMAND_ACCEL_FRONT_SS_PIN);

  // Power-on step 9
  FPGA_COMMAND_PORT &= ~(1<<FPGA_COMMAND_ACCELS_INIT_PIN);

  // Power-on step 10
  _delay_ms(200);

  // Power-on step 11
  SPI_DDR = (1<<SPI_MISO);
  // CPOL=0, CPHA=1 - Trailing (Falling) Edge
  // DORD = 1 - LSB first
  SPCR = _BV(SPE) | _BV(DORD) | _BV(CPHA);

  // Power-on step 12
  FPGA_ENABLE_PORT |= (1<<FPGA_ENABLE_PIN);

  // Set up laser port.
  DDRB |= 1<<PB6;
  
  // set up pump port
  DDRG |= 1<<PG0;
  
  // set up valve port
  DDRL |= 1<<PL6;
}

int main() {
  setup();

  byte i = 0;
  byte data;
  while (1) {
    if (SPDR == 0xa5) {
      SPDR = 0x00;
      readSpi();
      // Calibration routine.
      if (calibrator.isRunning()) {
        if (calibrator.isHit()) {
          if (calibrator.isBacking()) {
            writeSpi(calibrator.getBackoffCommand());
          } else {
            calibrator.startBacking();
            writeSpi(calibrator.getBackoffCommand());
          }
        } else if (calibrator.isBacking()) {
          calibrator.stop();
          // writeSpi((Command*) &sequenceRest[1]);
          writeSpiRest();
        } else {
          writeSpi(calibrator.getForwardCommand());
        }
      // If nothing left in the queue then send STOP.
      } else if (cmdQueue.isEmpty()) {
          // writeSpi((Command*) &sequenceRest[1]);
          writeSpiRest();
      // Some command is waiting to be processed.
      } else {
        Command* command = cmdQueue.popTail();
        // If movement command then send it to FPGA.
        if (command->type == Move) {
          writeSpi(command);
        // If not a movement command then execute it.
        } else {
          serialRead();
          implementationFunctions[command->type]();
        }
        // After a command has been sent to FPGA or directly executed peek at the
        // queue to see if there is another non-movement command is waiting next
        // and execute it too. Do until a movement command is next or no commands
        // left in the queue.
        while (!cmdQueue.isEmpty()) {
          command = cmdQueue.peekTail();
          if (command->type == Move) {
            break;
          } else {
            serialRead();
            cmdQueue.popTail();
            implementationFunctions[command->type]();
          }
        }
      }
      processSerialBuffer();
    } else {
      serialRead();
    }
  }
  return 0;
}
