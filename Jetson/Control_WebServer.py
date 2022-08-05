"""
Created on Thurs June 2 2022
@author: Hayley Wisman
Sets up a simple Flask web server to control and communicate with the batbot remotely via wifi.
"""

from flask import Flask, render_template, request, send_file
import glob
import os.path

app = Flask(__name__)

startDeform = False                              # Indicates whether to start pinna deformation


@app.route('/', methods=['GET', 'POST'])
def handle_request():
    global startDeform
    if request.method == "GET":                           # Send most recent data file if GET request is received
        filepath = r'C:\Users\hwisman21\Desktop\School\NEEC\batbot6\Jetson'   # Specify folder containing data files
        file_type = r'\*txt'                              # Specify file type of data files
        files = glob.glob(filepath + file_type)           # Look for txt files in the specified path
        recent_file = max(files, key=os.path.getctime)    # Retrieve the most recent txt file in path
        print(recent_file)
        return send_file(recent_file)
    if request.method == "POST":                 # Toggle start/stop pinna deformation if POST request received
        if not startDeform:
            startDeform = True
        if startDeform:
            startDeform = False
    return '200'                                 # Just return something to make the computer gods happy


if __name__ == '__main__':
    app.run(debug=True, port=8080)              # Run web server on port 8080 as it is not reserved for other services
