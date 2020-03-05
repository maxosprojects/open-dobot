#! /usr/bin/env python
"""
Simple demostration of open-dobot. High-level control via SDK.

It is assumed that no end effectors are installed on the arm (no gripper or sucker, etc.)
as end effector would crash into the desk Dobot stands on when folowing the examples below.
If you want to run the following examples with an end effector you would need to adjust all
coordinates in the commands correspondingly and make sure they are still in arm's in reachable area.

Refer to SDK to find the expected initial arm configuration.

Each move is split into a series of commands to send via driver to FPGA. Each command is
executed for 20ms. Dobot, thus, executes 50 commands per second.

The commands are queued in Arduino, hence when you stop this example dobot will continue
to execute until the queue (200 commands) is empty.

"""
import math
from dobot import Dobot
import time
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from PyQt5 import uic

# Maximum speed in mm/s
speed = 80
# Acceleration in mm/s^2
acceleration = 80

dobot = Dobot('COM17', debug=False, plot=True, rate=115200)
print("Turn ON!")

"""
# This loads the GUI from the .ui file that is created by QtDesigner. The .ui file should be in the same folder as this
# python file (or specify different path).
Ui_MainWindow, QtBaseClass = uic.loadUiType('DobotMainUi.ui')


# Here, a class is defined to represent the entire GUI. It is derived from a Qt class named QMainWindow, which
# corresponds to the GUI type specified in QtDesigner. All of the functional aspects (as opposed to design aspects) of
# the GUI are defined in this class. For example, what happens when a user presses a button.
class DobotGUIApp(QMainWindow):
    # class initialization function (initialize the GUI)
    def __init__(self, parent=None):
        # I'm a python noob, but I'm guessing this means initialize the parent class. I imagine all the super classes
        # have to be explicitly initialized.
        super(DobotGUIApp, self).__init__(parent)
        # This sets up the ui variable of this class to refer to the loaded .ui file.
        self.ui = Ui_MainWindow()
        # This call is required. Does whatever set up is required, probably gets references to the elements and so on.
        self.ui.setupUi(self)

        # Anything named after self.ui. (e.g. self.ui.x) means you are referring to an object name x that corresponds
        # to an element in the gui. Qt uses a signals and slots framework to define what happens in response to UI
        # events. You'll have to look that up if you're interested in it.

        ###
        # Connect gui elements in the .ui file to event handling functions.
        ###

        # connect move coordinates button clicked event to function to move to the coordinate specified
        self.ui.pushButtonMoveToCoordinate.clicked.connect(self.pushButtonMoveToCoordinate_clicked)
        # connect move to angles button clicked event to function to move to the angles specified
        self.ui.pushButtonMoveToAngles_2.clicked.connect(self.pushButtonMoveToAngles_clicked)

    def pushButtonMoveToCoordinate_clicked(self):

        # get moveTo coordinate text values from line edits
        moveToX = self.ui.lineEditMoveToX.text()
        moveToY = self.ui.lineEditMoveToY.text()
        moveToZ = self.ui.lineEditMoveToZ.text()

        # check that the values were not empty
        if moveToX == '' or moveToY == '' or moveToZ == '':
            self.show_a_warning_message_box('Missing a coordinate value.',
                                            'Check that you entered a value for each dimension.',
                                            'Invalid coordinate for move to command')
            return

        # convert values from string to float and ensure that the values entered were actually numbers
        try:
            moveToXFloat = float(moveToX)
            moveToYFloat = float(moveToY)
            moveToZFloat = float(moveToZ)
        except Exception as e:
            self.show_a_warning_message_box('Check that your coordinate values are numbers and not letters. The code '
                                            + 'error is shown below:',
                                            repr(e),
                                            'Coordinate value conversion to float error')
            return

        # call inverse kinematics function to convert from cartesian coordinates to angles for Dobot arm
        # moveToAngles is a list of angles (type float) with the following order: [base angle, upper arm angle, lower arm angle]
        # catch any errors (likely due to coordinates out of range being input) NEED TO ADDRESS THIS AT SOME POINT
        if moveToXFloat == 0 and moveToYFloat == 0 and moveToZFloat == 0:
            moveToXFloat = 217.117
            moveToYFloat = 0
            moveToZFloat = 229.430
        dobot.MoveWithSpeed(moveToXFloat, moveToYFloat, moveToZFloat, speed, acceleration)
        # if movement was successful, update the current position
        # note that float values are rounded to 3 decimal places for display and converted to strings
        moveToAngles = dobot.kinematics.anglesFromCoordinates(moveToXFloat, moveToYFloat, moveToZFloat)
        self.ui.labelBaseAngleValue.setText(str(round(moveToAngles[0] * 180 / math.pi, 3)))
        self.ui.labelUpperArmAngleValue.setText(str(round(moveToAngles[2] * 180 / math.pi, 3)))
        self.ui.labelLowerArmAngleValue.setText(str(round(moveToAngles[1] * 180 / math.pi, 3)))

        self.ui.labelCurrentXValue.setText(str(round(moveToXFloat, 3)))
        self.ui.labelCurrentYValue.setText(str(round(moveToYFloat, 3)))
        self.ui.labelCurrentZValue.setText(str(round(moveToZFloat, 3)))

    def pushButtonMoveToAngles_clicked(self):
        # get moveTo angle text values from line edits
        moveToBaseAngle = self.ui.lineEditMoveToBaseAngle.text()
        moveToUpperArmAngle = self.ui.lineEditMoveToUpperArmAngle_2.text()
        moveToLowerArmAngle = self.ui.lineEditMoveToLowerArmAngle_2.text()
        # check that the values were not empty
        if moveToBaseAngle == '' or moveToUpperArmAngle == '' or moveToLowerArmAngle == '':
            self.show_a_warning_message_box('Missing a angle value.',
                                            'Check that you entered a value for each angle.',
                                            'Invalid angles for move to command')
            return

        # convert values from string to float and ensure that the values entered were actually numbers
        try:
            moveToBaseAngleFloat = float(moveToBaseAngle)
            moveToUpperArmAngleFloat = float(moveToUpperArmAngle)
            moveToLowerArmAngleFloat = float(moveToLowerArmAngle)
        except Exception as e:
            self.show_a_warning_message_box('Check that your angle values are numbers and not letters. The code '
                                            + 'error is shown below:',
                                            repr(e),
                                            'Angle value conversion to float error')
            return

        # note that float values are rounded to 3 decimal places for display and converted to strings
        self.ui.labelBaseAngleValue.setText(str(round(moveToBaseAngleFloat, 3)))
        self.ui.labelUpperArmAngleValue.setText(str(round(moveToUpperArmAngleFloat, 3)))
        self.ui.labelLowerArmAngleValue.setText(str(round(moveToLowerArmAngleFloat, 3)))
        moveToLowerArmAngleFloat = moveToLowerArmAngleFloat * math.pi / 180
        moveToLowerArmAngleFloat = (math.pi / 2) - moveToLowerArmAngleFloat
        if moveToBaseAngleFloat == 0 and moveToUpperArmAngleFloat == 0 and moveToLowerArmAngleFloat == 0:
            moveToXYZFloat = [217.117, 0, 229.430]
        else:

            moveToXYZFloat = dobot.kinematics.coordinatesFromAngles(moveToBaseAngleFloat * math.pi / 180,
                                                                    moveToLowerArmAngleFloat,
                                                                    moveToUpperArmAngleFloat * math.pi / 180)

        dobot.moveArmToAngles(moveToBaseAngleFloat * math.pi / 180, moveToLowerArmAngleFloat,
                              moveToUpperArmAngleFloat * math.pi / 180, 20)

        # dobot.MoveWithSpeed(moveToXYZFloat[0], moveToXYZFloat[1], moveToXYZFloat[2], speed, acceleration)
        self.ui.labelCurrentXValue.setText(str(round(moveToXYZFloat[0], 3)))
        self.ui.labelCurrentYValue.setText(str(round(moveToXYZFloat[1], 3)))
        self.ui.labelCurrentZValue.setText(str(round(moveToXYZFloat[2], 3)))

    @staticmethod
    def show_a_warning_message_box(text, infoText, windowTitle):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Warning)
        msg.setText(text)
        msg.setInformativeText(infoText)
        msg.setWindowTitle(windowTitle)
        msg.exec()


# main function
if __name__ == '__main__':
    # These first three lines initialize the Qt application/GUI.
    app = QApplication(sys.argv)
    window = DobotGUIApp()
    # displays the GUI
    window.show()

    # write whatever set up or logic code you want to here.
    # Says to exit the whole code when the Qt application is closed. app.exec returns some value when qt app quits
    sys.exit(app.exec_())
"""

delay = 5
while delay > 0:
    print("Ready in: ", delay)
    delay -= 1
    time.sleep(1)

i = 0
coordinates = [0, 0, 0]
while True:
    print("Input the required coordinate or input 'h' to go home or 'q' to quit")
    if i == 0:
        print("Input X: ")
    elif i == 1:
        print("Input Y: ")
    elif i == 2:
        print("Input Z: ")
    key = input()
    if key == 'q':
        break
    elif key == 'h':
        coordinates[0] = 217
        coordinates[1] = 0
        coordinates[2] = 229
        print("Going Home: ", coordinates)
        dobot.MoveWithSpeed(coordinates[0], coordinates[1], coordinates[2], speed, acceleration)
        i = 0
    else:
        coordinates[i] = int(key)
        i = i + 1
        if i > 2:
            if coordinates[0] == 0 and coordinates[1] == 0 and coordinates[2] == 0:
                coordinates[0] = 217
                coordinates[1] = 0
                coordinates[2] = 229
            print("SENT: ", coordinates)
            print(dobot.driver.GetAccelerometers())
            dobot.MoveWithSpeed(coordinates[0], coordinates[1], coordinates[2], speed, acceleration)
            i = 0

# dobot.MoveWithSpeed(X, Y, Z, speed, acceleration)
# Draws a Rectangle
# dobot.MoveWithSpeed(220.0, 80, 5 + 40, speed, acceleration)
# dobot.MoveWithSpeed(220.0, -80.0, 5 + 40, speed, acceleration)
# dobot.MoveWithSpeed(270.0, -80.0, 5 + 40, speed, acceleration)
# dobot.MoveWithSpeed(270.0, 80.0, 5 + 40, speed, acceleration)
# dobot.MoveWithSpeed(220.0, 80, 5 + 40, speed, acceleration)
# dobot.Afif_PlotData()
#
# # Draws a Circle
# radius = 50
# theta = 0
# center_x = 220.0
# center_y = 80.0
# step = 10
# x = center_x + radius * math.cos(theta * math.pi / 180)
# y = center_y + radius * math.sin(theta * math.pi / 180)
# dobot.MoveWithSpeed(x, y, 10 + 40, speed, acceleration)
# while theta < 360:
#     x = center_x + radius * math.cos(theta * math.pi / 180)
#     y = center_y + radius * math.sin(theta * math.pi / 180)
#     dobot.MoveWithSpeed(x, y, 10 + 50, speed * 20, acceleration * 20)
#     theta = theta + step
# dobot.Afif_PlotData()
