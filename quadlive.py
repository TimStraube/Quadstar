"""
author: Tim Leonard Straube
email: tileone02@posteo.de
comment: webserver for quadcopter visualization
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

app = Flask(__name__)
testbench = Testbench()
try:
    sampleSerial = Quadserial()
    quadverbunden = True
except:
    print("Für Quadtest Quadcopter verbinden.")
    quadverbunden = False
quaternion = Quaternion()

@app.route('/', methods=['GET', 'POST', 'HEAD'])
def home():
    if request.method == 'POST':

        # Winkel in rad
        if config.Ordnername[1] == "P":
            (
                t, 
                norden, 
                osten, 
                unten, 
                rollen, 
                nicken, 
                gieren
            ) = testbench.testPID()
        elif config.Ordnername[1] == "M":
            (
                t, 
                norden, 
                osten, 
                unten, 
                rollen, 
                nicken, 
                gieren
            ) = testbench.testModell()
        else:
            print("Warnung! Invalider Reglertyp.")

        # Winkel in rad
        quad_state = jsonify({
            "t" : t,
            "norden" : norden,
            "osten" : osten,
            "unten" : unten,
            "rollen" : rollen,
            "nicken" : nicken,
            "gieren" : gieren
        })
        return quad_state
    else:
        testbench.reset()
        return render_template('simulation.html')
    
@app.route('/sensortest/', methods=['GET', 'POST', 'HEAD'])
def sensortest():
    if request.method == 'POST':
        if quadverbunden:
            sampleSerial.startEinmalsampler()
            samples = sampleSerial.aktuelleSample.tolist()
        else:
            samples = numpy.array([[0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0]])

        kardan = quaternion.quaternion2kardanwinkel([
            samples[16][0], 
            samples[17][0], 
            samples[18][0], 
            samples[19][0]
        ])
        # Winkel in rad
        quad_state = jsonify({
            "t" : 0,
            "norden" : 0,
            "osten" : 0,
            "unten" : 0,
            "rollen" : float(samples[5][0]) * 3.14 / 180.0,
            "nicken" : float(samples[4][0]) * 3.14 / 180.0,
            "gieren" : float(samples[3][0]) * 3.14 / 180.0
        })
        return quad_state
    else:
        testbench.reset()
        return render_template('sensortest.html')

if __name__ == '__main__':
    app.run(
        host="localhost", 
        debug=True
    )
