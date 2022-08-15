"""
Performs runs indefinitely, saving results and occasionally plotting them
"""

import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np
import serial
import serial.tools.list_ports
import struct
import time
import math
import os
from sys import executable
from subprocess import Popen, CREATE_NEW_CONSOLE
import Control_WebServer as webserver

# Number of runs between plot updates. Plotting almost doubles the
# duration of each run, so keep this large
PLOT_INTERVAL = 10

# COM port of the M4; leave None for cross-platform auto-detection
# PORT = "/dev/ttyTHS1"  #hardware pins 8 and 10
#PORT = None

class BatBot:
    """
    Bare minimum example (collect a single run):

        bb = BatBot()
        left, right = bb.collect_data()
        plt.plot(left)
        plt.plot(right)
        plt.show()

    """
    
    def __init__(self):
        """
        Connect to a device

        :param port: optional, COM port of M4

        probably add another serial connection
        """

        # Vendor and product ID of SAMD51 USB Host
        vid = 0x239a
        pid = 0x8031

        #Vendor and product ID of Teensy
        vid_teensy  = 0x16C0
        pid_teensy = 0x0483

        # Try to deduce the serial port
        #if not port:
        port1 = self.guess_port(pid, vid)

        port2 = self.guess_port(pid,vid)        #TODO wrong pid/vid for teensy

        # Connect to the device
        self.ser1 = serial.Serial(port1)
        self.ser2 = serial.Serial(port2)

        # Attempt to reset the device
        self.reset()

        print(f'Connected to M4 on {port1}')
        print(f'Connected to Teensy on {port2}')


    @staticmethod
    def guess_port(pid,vid):
        """
        Discover any locally connected boards

        :return: COM port name of discovered boards
        """

        #print ("Attempting to discover port of M4")
        
        # Vendor and product ID of SAMD51 USB Host
        VID = vid #0x239a
        PID = pid #0x8031

        # Try to detect any M4s by USB IDs
        available_ports = serial.tools.list_ports.comports()
        possible_ports = [port.device for port in available_ports \
                          if (port.vid == VID and port.pid == PID)]

        # Yell at the user if no M4 was found
        if not any(possible_ports):
            raise Exception('Board not found: verify that it is properly connected')

        return possible_ports[0]

    def reset(self):
        """
        Trigger a hardware reset using the serial's DTR; highly
        dependent on hardware configuration

        MODIFIED to Reset both M4 and Teensy serials. 
        """
        self.ser1.setDTR(False)
        self.ser2.setDTR(False)
        time.sleep(1)
        self.ser1.flushInput()
        self.ser2.flushInput()
        self.ser1.setDTR(True)
        self.ser2.setDTR(True)


    def writeM4(self, packet):
        self.ser1.write(packet) 

    #TODO write method for Teensies

    #as of now, the only read needed is for m4
    def read(self, length):
        return self.ser1.read(length)


    def _start_run(self):
        self.writeM4([0x10])

    def _wait_for_run_to_complete(self):
        while True:
            self.writeM4([0x20])

            if self.read(1) == b'\x01':
                return


    def _get_data(self, ch):
        self.writeM4([0x30 | ch])
        
        num_pages = self.read(1)[0]
        raw_data = self.read(num_pages * 8192)  # This contains the amount of data to be collected per page
        return [((y << 8) | x) for x, y in zip(raw_data[::2], raw_data[1::2])]


    def collect_data(self):
        """
        Perform a full data collection run

        :return: collected data split into separate channels
        """
        # Set the motion profile before data collection

        #TODO: If I (Henry) recall correctly, the goal was to move this out so we could implement more comprehensive unit testing
        # self.set_motion_profile(1,0,0,[0xF0]) 

        # Start the run by sending 0x10 to the M4 to signal start of data collection
        self._start_run()

        # Query the M4 status in a loop
        self._wait_for_run_to_complete()

        # Once the status is OK, get the data from the M4
        left_ch = self._get_data(ch=0)
        right_ch = self._get_data(ch=1)

        return (left_ch, right_ch)
#------------------------------------------- Main Loop Below -----------------------------------------------

if __name__ == '__main__':

    """
    Main thread of the code. Runs the plotting functionality.
    """

    # Subprocess call executes the web server script in a separate shell to run concurrently
    Popen([executable, 'Control_WebServer.py'])

    # Connect to M4 and make an instance of the BatBot object called "bb"
    bb = BatBot(port=PORT)

    print('-' * 60)

    # Ask for name of output folder
    print('Enter name of folder: ', end='')
    folder_name = input()

    print('-' * 60)
    print(f'Saving to {os.path.dirname(os.path.abspath(__file__))}')
    print('-' * 60)

    # Ask user if they want to limit number of runs
    # Number of runs also set in GUI
    print('Enter number of runs to perform (inf for continuous runs): ')
    n_runs = input()
    if n_runs != "inf":
        nruns = int(n_runs)

    # Create a subplot for each channel
    f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
    ax1.set_xlim([0, 10000])
    ax1.set_ylim([0, 4096])
    ax2.set_xlim([0, 10000])
    ax2.set_ylim([0, 4096])

    num_runs = 0
    trial_start = datetime.now()

    # Loop data collection indefinitely; press Ctrl-C and close the plot
    # to stop elegantly
    while True:
        try:
            run_start = time.time()*10**9

            # Start ear deformation during data collection
            #if webserver.startDeform:
                # need to form one cohesive function to call here that initiates pinna deformation

            # Collect data
            left, right = bb.collect_data()

            # Create output folder and file
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S%f')[:-3]
            output_folder = os.path.dirname(os.path.abspath(__file__)) + '/dataDst/'
            output_filename = timestamp + '.txt'
            output_path = output_folder + output_filename
            if not os.path.exists(output_folder):
                os.makedirs(output_folder)

            # Write output to file
            with open(output_path, 'w') as f:
                for data in left + right:
                    f.write('{}\n'.format(data))

            num_runs += 1

            # Periodically plot an incoming signal
            if num_runs % PLOT_INTERVAL == 0:
                elapsed = datetime.now() - trial_start

                # Leave a status message
                ax1.set_title('{} runs - {}'.format(num_runs, str(elapsed)[:-7]))
                ax2.set_title('{} runs/min'.format(int(num_runs/max(elapsed.seconds,1)*60)))

                # Clear previous lines (for speed)
                ax1.lines = ax2.lines = []

                # Plot
                ax1.plot(left)
                ax2.plot(right)

                # Show the plot without blocking (there's no separate UI
                # thread)
                plt.show(block=False)
                plt.pause(0.001)

            # *** Future signal processing and other kinds of things can go here in the code ***
            if num_runs == nruns:
                break

        except KeyboardInterrupt:
            print('Interrupted')
            break

    print('-' * 60)

    elapsed = datetime.now() - trial_start
    print(f'{num_runs} runs took {elapsed}')

