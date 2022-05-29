#!/usr/bin/python
import time
from Jetson import Batbot_Control as BCTL

print("Servo Test Entered. Development Completed, running test")
#time.sleep(3)
 

#---------------------- Main Code Below ---------------------------------

bb = BCTL.BatBot() #this code is bring run on the Jetson, so there is no need for an external computer to "need to connect to the Jetson" since it is stand alone

print("Moving Servo A")
bb.moveEar(180, 'A')
time.sleep(3)
bb.moveEar(0, 'A')

time.sleep(5)

print("Moving Servo B")
bb.moveEar(180, 'B')
time.sleep(3)
bb.moveEar(0, 'B')

print("Servo Test has run to completion")

