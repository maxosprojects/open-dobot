from dobot import DobotDriver

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
driver.Close()
