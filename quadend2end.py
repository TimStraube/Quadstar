"""
author: Tim Leonard Straube
email: hi@optimalpi.de
"""

import gymnasium
import numpy
import config as config
import random
import matplotlib.pyplot as plt
from gymnasium import spaces
from quaternion import Quaternion
from quad import Quadcopter

class Quadend2end(gymnasium.Env):
    def __init__(self):
        """Famara Gymnasium Umwelt zum Training des Ende-zu-Ende-Modells
        """
        super(Quadend2end, self).__init__()
        # Quaternionklasse welche Methoden zur Umrechnung von Quaternionen und Kardanwinkeln zur Verfügung stellt
        self.quaternion = Quaternion()
        # Initialisierung der Quadcopterklasse
        self.quad = Quadcopter()
        # Initialisierung des Beobachtungsvektors
        self.observation = numpy.zeros(
            9, 
            dtype=numpy.float32
        )
        # Definition des Aktionsraums
        self.action_space = spaces.MultiDiscrete(
            numpy.concatenate([
                config.actionspace_end2end[0] * numpy.ones(4),
                [config.actionspace_end2end[1]]
            ])
        )
        # Definition des Beobachtungsraums
        self.observation_space = spaces.Box(
            low = -10,
            high = 10,
            shape = (9,),
            dtype = numpy.float32
        )
        # Initalisierung der Quadcopterparameter
        self.quad.reset()
        self.reset()

    def reset(self, seed = None):
        self.velocity_setsarray_norden = []
        self.velocity_setsarray_osten = []
        self.velocity_setsarray_unten = []
        self.array_norden = []
        self.array_osten = []
        self.array_unten = []
        self.omega1_all = []
        self.omega2_all = []
        self.omega3_all = []
        # Initalisierung der Zeitvariable
        self.t = 0
        # Schrittweite
        self.step_size = config.step_size
        # Schritt der aktuellen Episode
        self.step_current = 0
        # Sollgeschwindigkeit
        self.velocity_set = numpy.array([
            random.randint(-1, 1),
            random.randint(-1, 1),
            0
        ])

        self.quad.reset()
        # Drehlage = [
        #   Gierwinkel (rad),
        #   Nichwinkel (rad),
        #   Rollwinkel (rad)
        # ]
        self.attitude = self.quaternion.quaternion2cardan(
            self.quad.zustand[3:7]
        )
        
        # Beobachtung bestehend aus der Drehlage, der Geschwindigkeit und der Sollgeschwindigkeit
        self.observation[:] = numpy.float32(
            numpy.concatenate([
                self.attitude,
                self.quad.zustand[7:10], 
                self.velocity_set
            ])
        )

        return self.observation, {}
    
    def step(self, action):
        terminated = False
        truncated = False

        # Das Umweltmodell wird geupdated so oft wie es der Agent will
        for _ in range(action[4] + 1):
            self.step_current += 1
            # Update des Quadcopterzustands
            self.quad.update(
                self.t, 
                action
            )
            # Update der Zeit
            self.t += config.step_size

        # Drehlage = [
        #   Gierwinkel (rad),
        #   Nichwinkel (rad),
        #   Rollwinkel (rad)
        # ]
        self.attitude = self.quaternion.quaternion2cardan(
            self.quad.zustand[3:7]
        )             

        self.velocity_setsarray_norden.append(
            self.t * self.velocity_set[0]
        )
        self.velocity_setsarray_osten.append(
            self.t * self.velocity_set[1]
        )
        self.velocity_setsarray_unten.append(
            self.t * self.velocity_set[2]
        )
        self.array_norden.append(self.quad.zustand[0])
        self.array_osten.append(self.quad.zustand[1])
        self.array_unten.append(self.quad.zustand[2])
        self.omega1_all.append(self.attitude[0])
        self.omega2_all.append(self.attitude[1])
        self.omega3_all.append(self.attitude[2])

        # Berechnen der Belohnung im aktuellen Zustand
        belohnung = self.quad.belohnung()

        # if numpy.max(numpy.abs(self.state[0:3])) > 2:
        #     truncated = True

        # Beenden der Episode wenn die maximale Episodenlänge erreicht wurde
        if self.step_current >= (config.Quadtrain_finaler_Zeitpunkt / config.step_size):
            truncated = True
            if config.render_modell:
                # Zeichnen der Positions- und Drehlagetrajektorien
                print("\033c", end='')
                self.render()

        # Beobachtung bestehend aus der Drehlage, der Geschwindigkeit und der Sollgeschwindigkeit
        self.observation[:] = numpy.float32(
            numpy.concatenate([
                self.attitude,
                self.quad.zustand[7:10], 
                self.velocity_set
            ])
        )

        return (
            self.observation, 
            belohnung, 
            terminated, 
            truncated, 
            {}
        )
    
    def render(self):
        f, (ax1, ax2, ax3, ax4, ax5, ax6) = plt.subplots(
            1, 
            6, 
            sharex = True
        )
        ax1.plot(self.array_norden)
        ax1.plot(self.velocity_setsarray_norden)
        ax1.set_title('North/m')
        ax2.plot(self.array_osten)
        ax2.plot(self.velocity_setsarray_osten)
        ax2.set_title('East/m')
        ax3.plot(self.array_unten)
        ax3.plot(self.velocity_setsarray_unten)
        ax3.set_title('Down/m')
        ax4.plot(self.omega1_all)
        ax4.set_title('Roll/rad')
        ax5.plot(self.omega2_all)
        ax5.set_title('Pitch/rad')
        ax6.plot(self.omega3_all)
        ax6.set_title('Yaw/rad')

        plt.show()