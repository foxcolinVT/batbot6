"""
Created on Thurs June 2 2022
@author: Hayley Wisman
Sets up a web server to control the batbot remotely via wifi
"""

from flask import Flask, render_template, request, send_file

app = Flask(__name__)

func = ''


@app.route('/', methods=['GET', 'POST'])
def render():
    if request.method == "GET":
        return send_file('20181111_121335587.txt')
    return render_template('index.html')        # Return the webpage


if __name__ == '__main__':
    app.run(debug=True, port=8080)               # Run web server on port 8080 as it is not reserved for other services