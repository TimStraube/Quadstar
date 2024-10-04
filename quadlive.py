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
from quadtest import Testbench
from quadserial import Quadserial
from quaternion import Quaternion

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
            print("Warnung! Invalider Reglertyp.")

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
    
@app.route('/sensortest/', methods=['GET', 'POST', 'HEAD'])
def sensortest():
    if request.method == 'POST':
        if quad_connected:
            sampleSerial.startEinmalsampler()
            samples = sampleSerial.sample_current.tolist()
        else:
            samples = numpy.array([[0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0]])

        # kardan = quaternion.quaternion2cardan([
        #     samples[16][0], 
        #     samples[17][0], 
        #     samples[18][0], 
        #     samples[19][0]
        # ])

        # Winkel in rad
        quad_state = jsonify({
            "t" : 0,
            "north" : 0,
            "east" : 0,
            "down" : 0,
            "roll" : float(samples[5][0]) * deg2rad,
            "pitch" : float(samples[4][0]) * deg2rad,
            "yaw" : float(samples[3][0]) * deg2rad
        })
        return quad_state
    else:
        testbench.reset()
        return render_template('sensortest.html')

if __name__ == '__main__':
    app.run(host="localhost", debug=True)
