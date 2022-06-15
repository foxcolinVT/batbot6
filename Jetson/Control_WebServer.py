"""
Created on Thurs June 2 2022
@author: Hayley Wisman
Sets up a web server to control the batbot remotely via wifi
"""

from flask import Flask, render_template, request

app = Flask(__name__)

@app.route('/', methods=['GET', 'POST'])
def render():
    if request.method == "POST":
        func = request.form["function"]
        #if func == "startData":
            # Send opcode to start data acquisition?
        #return func + " Hello"                               this line for debugging purposes
    return render_template('index.html')

if __name__ == '__main__':
    app.run(debug=True, port=8080)           # Run web server on port 8080 as it is not reserved for other services