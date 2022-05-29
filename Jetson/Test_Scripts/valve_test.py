#!/usr/bin/python
import time
import Batbot_Control as BCTL

print("Valve Test Entered. This unit testing case is still under development")
time.sleep(3)


#---------------------- Main Code Below ---------------------------------

bb = BCTL.BatBot() #this code is bring run on the Jetson, so there is no need for an external computer to "need to connect to the Jetson" since it is stand alone


valves = ["A", "B", "C", "D", "E", "F", "G", "H"]
for x in valves:
    print(x)
    bb.moveValve(x)
    time.sleep(2)


print("Servo Test has run to completion")
