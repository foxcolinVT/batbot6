# -*- coding: utf-8 -*-
"""
Created on Thu Jun 16 13:29:29 2022

@author: Colin Fox
"""

import serial
import time
import serial.tools.list_ports

def connect(port=None):
    """
    Connect to a device
    :param port: optional, COM port of M4
    """
    # Try to deduce the serial port
    if not port:
        port = guess_port()
    # Connect to the device
    ser = serial.Serial(port, timeout=1)
    #this makes it so if it starts to recieve data but 
    #stops for 0.2 seconds the read will timeout
    ser.inter_byte_timeout = 0.2
    # Attempt to reset the device
    reset(ser)
    print(f'Connected to M4 on {port}')
    return ser
    
def reset(ser):
    """
    Trigger a hardware reset using the serial's DTR; highly
    dependent on hardware configuration
    MIGHT NOT NEED
    """
    ser.setDTR(False)
    time.sleep(1)
    ser.flushInput()
    ser.setDTR(True)
    
def guess_port():
    """
    Discover any locally connected M4s
    :return: COM port name of discovered M4
    """        
    # Vendor and product ID of SAMD51 USB Host (NEED TO CHANGE FOR TEENSY)
    VID = 0x239a
    PID = 0x8031
    # Try to detect any M4s by USB IDs
    available_ports = serial.tools.list_ports.comports()
    possible_ports = [port.device for port in available_ports \
                        if (port.vid == VID and port.pid == PID)]
    # Yell at the user if no M4 was found
    if not any(possible_ports):
        raise Exception('M4 not found: verify that it is properly connected')
    return possible_ports[0]
    

if __name__ == '__main__':
    ser = connect()
    #print init shit
    print('Input commands as 0x(hexval), with hexvals being no larger then one byte. Type "exit" to quit application')
    
    currCommand = input("Command: ")
    while(currCommand!='exit'):
        ser.write([int(currCommand, 0)])
        #add something for serial output
        
        #MIGHT WANT TO CHANGE IF NOT NULL TERMINATED
        cc=str(ser.readline())
        print(cc[2:][:-5])
        currCommand = input("Next command: ")
    ser.close()
    quit()          #hopefully this does what i want it to do
    
    #while loop, 
    
