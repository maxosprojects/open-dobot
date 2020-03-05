#! /usr/bin/env python
'''
Simple demostration of open-dobot. Low-level control via driver.

The commands are queued in Arduino, hence when you stop this example dobot will continue
to execute until the queue (200 commands) is empty.

There are 50 commands/second dobot executes (50Hz). So if you send only 50 commands dobot
will execute for exactly one second.

Number of steps is determined by the frequency and the execution window (20ms = 0.02s).

So, if you send only one command "50Hz" then dobot makes exacly one step and stops.

number_of_steps = 50Hz * 20ms = 50 * 0.02 = 1

For 600Hz:
number_of_steps = 600Hz * 20ms = 600 * 0.02 = 12

'''

from dobot import DobotDriver
import time

# driver = DobotDriver('COM4')
driver = DobotDriver('COM4')
driver.Open()
successes = 0

i = 0
while True:
    ret = driver.isReady()
    if ret[0] and ret[1]:
        successes += 1
        print(successes)
    if successes > 10:
        print("Dobot ready!")
        break
    if i > 100:
        raise Exception('Comm problem')

gripper = 480
toolRotation = 0


steps1 = driver.stepsToCmdVal(20)
steps2 = driver.stepsToCmdVal(0)
steps3 = driver.stepsToCmdVal(0)
driver.SetCounters(0, 0, 0)

print("Executing 20 commands with steps 27, -3, -14. Expecting GetCounters() to return:", 27 * 20, -3 * 20, -14 * 20)
errors = 0
for i in range(20):
    ret = (0, 0)
    while not ret[1]:
        ret = driver.Steps(steps1, steps2, steps3, 0, 0, 0, gripper, toolRotation)
print(driver.GetCounters())
# exit(0)