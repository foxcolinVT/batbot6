#!/usr/bin/python
import time
import Batbot_Control as BCTL

print("Stepper Test Entered. This unit testing case is still under development")
time.sleep(3)

bb = BCTL.BatBot() #this code is bring run on the Jetson, so there is no need for an external computer to "need to connect to the Jetson" since it is stand alone

#---------------------- Main Code Below ---------------------------------


print("Moving Right Stepper")
bb.moveActuator('A', 120, 0)
time.sleep(3)
bb.moveActuator('A', 120, 1)

time.sleep(5)

print("Moving Left Stepper")
bb.moveActuator('B', 120, 0)
time.sleep(3)
bb.moveActuator('B', 120, 1)

print("Servo Test has run to completion")
