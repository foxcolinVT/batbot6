"""
Created on Thurs June 2 2022
@author: Hayley Wisman
Sets up a web server to control the batbot remotely via wifi
"""

from flask import Flask, render_template, request, send_file

app = Flask(__name__)

startDeform = False                              # Indicates whether to start pinna deformation


@app.route('/', methods=['GET', 'POST'])
def handle_request():
    global startDeform
    if request.method == "GET":                  # Send most recent data file if GET request is received
        return send_file('example_data.txt')
    if request.method == "POST":                 # Toggle start/stop pinna deformation if POST request received
        if not startDeform:
            startDeform = True
        if startDeform:
            startDeform = False
    return '200'                                 # Just return something to make the computer gods happy


if __name__ == '__main__':
    app.run(debug=True, port=8080)              # Run web server on port 8080 as it is not reserved for other services
