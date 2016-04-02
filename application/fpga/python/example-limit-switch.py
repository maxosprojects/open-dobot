#! /usr/bin/env python
'''
An example running a calibration procedure using a limit switch/photointerrupter connected to a any
of the unused pins on Arduino, enabling/disabling pullup, specifying the switch type (normal LOW or
normal HIGH), forward joint rotation direction (towards the switch), forward and backward speeds.

Refer to DobotDriver.CalibrateJoint() for details.

This example works well with a photointerrupter shown on open-dobot/docs/images/interrupter*.jpg
It is cheap and easily mounted and yet sturdy and reliably. Requires minimum effort to get accurate
calibration for Joint1 (base), which originally has no sensors.
You can get an interrupter here:
https://www.sparkfun.com/products/9299
https://www.sparkfun.com/products/9322

'''

from DobotDriver import DobotDriver

driver = DobotDriver('/dev/tty.usbmodem1421')
driver.Open()

# Rotate base CW at 400 steps/s until limit switch is hit. Then retract CCW at 50 steps/s
# until switch is released and stop.
# Switch is expected to be connected (soldered) to pin D8 and pulled up (HIGH) externally (with a
# resistor to 5V supply) or be an active device (like a photointerrupter). Pullup is not enabled on that pin.

print driver.CalibrateJoint(1, driver.freqToCmdVal(1000), driver.freqToCmdVal(50), 1, 5, 1, 0)
