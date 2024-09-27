# -*- coding: utf-8 -*-
"""
author: Tim Leonard Straube
organization: HTWG Konstanz
email: ti741str@htwg-konstanz.de
comment: Famara Gymnasium Umwelt für das Lernen der PID-Parameter
"""

import gymnasium
import numpy
import config
import random
import matplotlib.pyplot as plt
from gymnasium import spaces
from wind import Wind
from quaternion import Quaternion
from scipy.integrate import ode
from quadregler import Regler
from quad import Quadcopter


class Quadpid(gymnasium.Env):
    belohnung_alt = 0

    def __init__(self):
        """Famara Gymnasium Umwelt zum Training der PID-Parameter des Quadcopterreglers
        """
        super(Quadpid, self).__init__()
        # Quaternionklasse welche Methoden zur Umrechnung von Quaternionen und Kardanwinkeln zur Verfügung stellt
        self.quaternion = Quaternion()
        # Initialisierung der Quadcopterklasse
        self.quad = Quadcopter()
        # Initialisierung des Beobachtungsvektors
        self.beobachtung = numpy.zeros(
            9, 
            dtype=numpy.float32
        )
        # Definition des Aktionsraums
        self.action_space = spaces.MultiDiscrete(
            [10, 10, 10, 10, 10, 10]
        )
        # Definition des Beobachtungsraums
        self.observation_space = spaces.Box(
            low = -15,
            high = 15,
            shape = (9, ),
            dtype = numpy.float32
        )
        # Initalisierung der Quadcopterparameter

        self.quad.reset()
        self.reset()

    def reset(self, seed = None):
        """Zurücksetzten des Quadcopters und Initalisierung von Variablen
        """
        self.sollgeschwindigkeitsarray_norden = []
        self.sollgeschwindigkeitsarray_osten = []
        self.sollgeschwindigkeitsarray_unten = []
        self.array_norden = []
        self.array_osten = []
        self.array_unten = []
        self.omega1_all = []
        self.omega2_all = []
        self.omega3_all = []
        # Initalisierung der Zeitvariable
        self.t = 0
        # Schrittweite
        self.Ts = config.Schrittweite
        # Schritt der aktuellen Episode
        self.schritt = 0
        # Sollgeschwindigkeit
        self.quad.sollgeschwindigkeit = numpy.array([
            1,
            0,
            0
        ])
        # Reset des Quadcopterzustands
        self.quad.reset()
        # Initialisierung des PID-Reglers
        self.controller = Regler(self.quad)
        # Berechnung der ersten Motorbefehle mit dem PID-Regler
        self.controller.regelschritt(
            self.quad, 
            config.Schrittweite,
            self.quad.sollgeschwindigkeit
        )
        # Windmodell
        self.wind = Wind('NONE', 0.0, 0, -15)
        # Initalisierung der Differenzialgleichungen des Quadcopters
        self.integrator = ode(
            self.quad.zustandsänderung
        ).set_integrator(
            'dopri5',
            first_step='0.00005',
            atol='10e-6',
            rtol='10e-6'
        )
        # Definition der Anfangsbedingungen
        self.integrator.set_initial_value(
            self.quad.zustand, 
            config.Episodenstart
        )
        # Drehlage = [
        #   Gierwinkel (rad),
        #   Nichwinkel (rad),
        #   Rollwinkel (rad)
        # ]
        self.drehlage = self.quaternion.quaternion2kardanwinkel(
            self.quad.zustand[3:7]
        )   
        # Beobachtung bestehend aus der Drehlage, der Geschwindigkeit und der Sollgeschwindigkeit
        self.beobachtung[:] = numpy.float32(
            numpy.concatenate([
                self.drehlage,
                self.quad.zustand[7:10], 
                self.quad.sollgeschwindigkeit
            ])
        )

        return self.beobachtung, {}

    def step(self, aktion):
        """
        """
        terminated = False
        truncated = False
        # Transformation der Aktion in das Sollbereich
        # Anpassen der PID-Parameter mit der Aktion des Agentens
        self.controller.vel_P_gain[0] = aktion[0] / 5 + 0.01
        self.controller.vel_P_gain[1] = aktion[0] / 5 + 0.01
        self.controller.vel_P_gain[2] = aktion[1] / 5 + 0.01
        self.controller.vel_I_gain[0] = aktion[2] / 5 + 0.01
        self.controller.vel_I_gain[1] = aktion[2] / 5 + 0.01
        self.controller.vel_I_gain[2] = aktion[3] / 5 + 0.01
        self.controller.vel_D_gain[0] = aktion[4] / 5 + 0.01
        self.controller.vel_D_gain[1] = aktion[4] / 5 + 0.01
        self.controller.vel_D_gain[2] = aktion[5] / 5 + 0.01
        # self.controller.attitute_p_gain[0] = aktion[6] / 5 + 0.01
        # self.controller.attitute_p_gain[1] = aktion[6] / 5 + 0.01
        # self.controller.attitute_p_gain[2] = aktion[7] / 5 + 0.01
        # self.controller.Pp = aktion[8] / 5 + 0.01
        # self.controller.Dp = aktion[9] / 5 + 0.01
        # self.controller.Pq = self.controller.Pp
        # self.controller.Dq = self.controller.Dp
        # self.controller.Pr = aktion[10] / 5 + 0.01
        # self.controller.Dr = aktion[11] / 5 + 0.01
        # self.controller.rate_P_gain[0] = aktion[12] / 5 + 0.01
        # self.controller.rate_P_gain[1] = aktion[13] / 5 + 0.01
        # self.controller.rate_P_gain[2] = aktion[14] / 5 + 0.01
        # self.controller.rate_D_gain[0] = aktion[15] / 5 + 0.01
        # self.controller.rate_D_gain[1] = aktion[16] / 5 + 0.01
        # self.controller.rate_D_gain[2] = aktion[17] / 5 + 0.01

        belohnung = 0

        while not (terminated or truncated):
            self.schritt += 1

            self.quad.update(
                self.t, 
                self.controller.motorbefehle, 
                self.wind
            )
            
            self.t += self.Ts
            
            self.quad.sollgeschwindigkeit = numpy.array([
                random.randint(0, 2),
                0,
                0
            ])

            self.controller.regelschritt(
                self.quad,
                self.Ts,
                self.quad.sollgeschwindigkeit
            )

            # Berechnen der Belohnung im aktuellen Zustand
            belohnung += self.quad.belohnung()

            if (self.t > config.Episodenende):
                truncated = True
                terminated = True
        # Drehlage = [
        #   Gierwinkel (rad),
        #   Nichwinkel (rad),
        #   Rollwinkel (rad)
        # ]
        self.drehlage = self.quaternion.quaternion2kardanwinkel(
            self.quad.zustand[3:7]
        )             

        # print(belohnung)

        # print(self.belohnung_alt)

        # self.sollgeschwindigkeitsarray_norden.append(
        #     self.t * self.sollgeschwindigkeit[0]
        # )
        # self.sollgeschwindigkeitsarray_osten.append(
        #     self.t * self.sollgeschwindigkeit[1]
        # )
        # self.sollgeschwindigkeitsarray_unten.append(
        #     self.t * self.sollgeschwindigkeit[2]
        # )
        # self.array_norden.append(self.quad.zustand[0])
        # self.array_osten.append(self.quad.zustand[1])
        # self.array_unten.append(self.quad.zustand[2])
        # self.omega1_all.append(self.drehlage[0])
        # self.omega2_all.append(self.drehlage[1])
        # self.omega3_all.append(self.drehlage[2])

        # if numpy.max(numpy.abs(self.zustand[0:3])) > 2:
        #     truncated = True

        # if belohnung != 0:
        #     belohnung = belohnung - self.belohnung_alt
        #     self.belohnung_alt = belohnung
        # else:
        #     belohnung = -1e5

        # self.render()

        # Beobachtung bestehend aus der Drehlage, der Geschwindigkeit und der Sollgeschwindigkeit
        self.beobachtung[:] = numpy.float32(
            numpy.concatenate([
                self.drehlage,
                self.quad.zustand[7:10], 
                self.quad.sollgeschwindigkeit
            ])
        )

        return (
            self.beobachtung, 
            belohnung, 
            terminated, 
            truncated, 
            {}
        )

    def render(self):
        """
        """
        f, (ax1, ax2, ax3, ax4, ax5, ax6) = plt.subplots(
            1, 
            6, 
            sharex = True
        )
        ax1.plot(self.array_norden)
        ax1.plot(self.quad.sollgeschwindigkeitsarray_norden)
        ax1.set_title('North/m')
        ax2.plot(self.array_osten)
        ax2.plot(self.quad.sollgeschwindigkeitsarray_osten)
        ax2.set_title('East/m')
        ax3.plot(self.array_unten)
        ax3.plot(self.quad.sollgeschwindigkeitsarray_unten)
        ax3.set_title('Down/m')
        ax4.plot(self.omega1_all)
        ax4.set_title('Roll/rad')
        ax5.plot(self.omega2_all)
        ax5.set_title('Pitch/rad')
        ax6.plot(self.omega3_all)
        ax6.set_title('Yaw/rad')

        plt.show()