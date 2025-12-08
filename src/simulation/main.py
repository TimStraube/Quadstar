"""
author: Tim Leonard Straube
email: hi@optimalpi.de
"""

import numpy
from flask import Flask
from flask import render_template
from flask import request
from flask import jsonify
from simulation.test import Testbench
from util.quaternion import Quaternion

deg2rad = numpy.pi / 180.0

app = Flask(__name__)
testbench = Testbench()
quad_connected = False
quaternion = Quaternion()

@app.route('/', methods=['GET', 'POST', 'HEAD'])
def home():
    if request.method == 'POST':
        # Führe einen einzelnen Simulationsschritt aus
        (
            t, 
            north, 
            east, 
            down, 
            roll, 
            pitch, 
            yaw
        ) = testbench.simulationStep()

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
