"""
Created on Thurs June 2 2022
@author: Hayley Wisman
Sets up a web server to control the batbot remotely via wifi
"""

from flask import Flask, render_template

app = Flask(__name__)

@app.route('/')
def render():
    return render_template('index.html')

@app.route('/A')
def start():
    return render_template('index.html')


if __name__ == '__main__':
    app.run(debug=True, port=8080)           # Run web server on port 8080 as it is not reserved for other services