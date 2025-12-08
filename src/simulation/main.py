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

@app.route('/', methods=['GET', 'POST', 'HEAD', 'OPTIONS'])
def home():
    if request.method == 'OPTIONS':
        # Handle preflight request
        response = jsonify({'status': 'ok'})
        response.headers.add('Access-Control-Allow-Origin', '*')
        response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
        response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
        return response
    elif request.method == 'POST':
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
            "north" : north,  # Back to original naming for compatibility
            "east" : east,    
            "down" : down, 
            "roll" : roll,
            "pitch" : pitch,
            "yaw" : yaw
        })
        # Add CORS headers
        quad_state.headers.add('Access-Control-Allow-Origin', '*')
        quad_state.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
        quad_state.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
        return quad_state
    else:
        testbench.reset()
        return render_template('simulation.html')

if __name__ == '__main__':
    app.run(host="localhost", debug=True)
