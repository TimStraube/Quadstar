"""
author: Tim Leonard Straube
email: hi@optimalpi.de
"""

import config
import numpy
from flask import Flask
from flask import render_template
from flask import request
from flask import jsonify
from simulation.test import Testbench
from util.serial import Quadserial
from util.quaternion import Quaternion

deg2rad = numpy.pi / 180.0

app = Flask(__name__)
testbench = Testbench()
try:
    sampleSerial = Quadserial()
    quad_connected = True
except:
    print("Für Quadtest Quadcopter verbinden.")
    quad_connected = False
quaternion = Quaternion()

@app.route('/', methods=['GET', 'POST', 'HEAD'])
def home():
    if request.method == 'POST':

        # Winkel in rad
        if config.model_id[1] == "P":
            (
                t, 
                north, 
                east, 
                down, 
                roll, 
                pitch, 
                yaw
            ) = testbench.testPID()
        elif config.model_id[1] == "M":
            (
                t, 
                north, 
                east, 
                down, 
                roll, 
                pitch, 
                yaw
            ) = testbench.testModell()
        else:
            print("Invalid controller type.")

        # t [s], NED [m], Attitude [rad]
        quad_state = jsonify({
            "t" : t,
            "north" : north,
            "east" : east,
            "down" : down,
            "roll" : roll,
            "pitch" : pitch,
            "yaw" : yaw
        })
        return quad_state
    else:
        testbench.reset()
        return render_template('simulation.html')

if __name__ == '__main__':
    app.run(host="localhost", debug=True)
