import config
import numpy
import random
import time
import matplotlib.pyplot as plt
from stable_baselines3 import PPO
from quad import Quadcopter
from wind import Wind
from quadregler import Regler
from utils import utils
from quaternion import Quaternion

class Testbench():
    def __init__(self):
        """Testbench zur Evaluation von Reglern
        """
        self.quaternion = Quaternion()
        self.initModell()
        self.reset()

    def initModell(self):
        """Modell laden
        """
        model_pfad = (
            f"./Modelle/{config.Ordnername}/best_model.zip"
        )
        self.model = PPO.load(model_pfad)

    def reset(self):
        """Quadcopter zurücksetzen
        """
        self.quad = Quadcopter()
        self.quad.reset()
        self.t = 0
        self.wind = Wind('NONE', 0.0, 0, 0)
        self.pid_gesetzt = False

        self.controller = Regler(
            self.quad
        )
        # Generate First Commands
        self.controller.regelschritt(
            self.quad, 
            config.Schrittweite,
            self.quad.sollgeschwindigkeit
        )
        self.drehlage = [
            0, 0, 0
        ]
        self.x_soll = [0]
        self.y_soll = [0]
        self.z_soll = [0]
        self.geschwindigkeitssoll_x = []
        self.geschwindigkeitssoll_y = []
        self.geschwindigkeitssoll_z = []
        self.t_all = []
        self.x_all = []
        self.y_all = []
        self.z_all = []
        self.geschwindigkeitsvektor_x = []
        self.geschwindigkeitsvektor_y = []
        self.geschwindigkeitsvektor_z = []
        self.omega1_all = []
        self.omega2_all = []
        self.omega3_all = []
        self.x = 0
        self.y = 0
        self.z = 0
        self.omega1 = 0
        self.omega2 = 0
        self.omega3 = 0

        self.beobachtung = numpy.zeros(
            9, 
            dtype=numpy.float32
        )
        # Sollvektor [Norden (m), Osten (m), Unten (m)]
        self.sollgeschwindigkeit = [1, 0, 0]
        
    def vorhersage(self):
        self.drehlage = self.quaternion.quaternion2kardanwinkel(
            self.quad.zustand[3:7]
        )             

        self.beobachtung[:] = numpy.float32(
            numpy.concatenate([
                self.drehlage,
                self.quad.zustand[7:10], 
                self.sollgeschwindigkeit
            ])
        )

        return self.model.predict(
            self.beobachtung
        )[0]
    
    def vorhersage(self):
        self.drehlage = self.quaternion.quaternion2kardanwinkel(
            self.quad.zustand[3:7]
        )             

        self.beobachtung[:] = numpy.float32(
            numpy.concatenate([
                self.drehlage,
                self.quad.zustand[7:10], 
                self.sollgeschwindigkeit
            ])
        )

        return self.model.predict(
            self.beobachtung, deterministic=True
        )[0]

    def testPID(self):
        """Methode zur Simulation mit den gelernten PID-Paramter oder optional den Default PID-Paramtern
        """
        aktion = self.vorhersage()
        print(aktion)
        # aktion = (
        #     (config.Aktionsinterval_PID[1] / 2) * (aktion + config.Aktionsinterval_PID[0] + 1)
        # )
        aktion = 9.9 * aktion + 11
        self.controller.vel_P_gain[0] = 1 # aktion[0] 
        self.controller.vel_P_gain[1] = 1 # aktion[0] 
        self.controller.vel_P_gain[2] = 1 # aktion[1] 

        self.controller.vel_I_gain[0] = 1 # aktion[2]
        self.controller.vel_I_gain[1] = 1 # aktion[2]
        self.controller.vel_I_gain[2] = 1 # aktion[3]

        self.controller.vel_D_gain[0] = 0.1 # aktion[4]
        self.controller.vel_D_gain[1] = 0.1 # aktion[4] 
        self.controller.vel_D_gain[2] = 0.1 # aktion[5]

        # self.controller.attitute_p_gain[0] = aktion[6]
        # self.controller.attitute_p_gain[1] = aktion[6]
        # self.controller.attitute_p_gain[2] = aktion[7]

        # self.controller.Pp = aktion[8]
        # self.controller.Dp = aktion[9]
        # self.controller.Pq = self.controller.Pp
        # self.controller.Dq = self.controller.Dp
        # self.controller.Pr = aktion[10]
        # self.controller.Dr = aktion[11]

        # self.controller.rate_P_gain[0] = aktion[12]
        # self.controller.rate_P_gain[1] = aktion[13]
        # self.controller.rate_P_gain[2] = aktion[14]

        # self.controller.rate_D_gain[0] = aktion[15]
        # self.controller.rate_D_gain[1] = aktion[16]
        # self.controller.rate_D_gain[2] = aktion[17]
            
        aktiv = 1
        while (aktiv):

            # Norden, Osten, Unten
            self.sollgeschwindigkeit = [
                0 + 1 * numpy.sin(1 * self.t), 
                0,
                0
            ]

            self.quad.update(
                self.t, 
                self.controller.motorbefehle, 
                self.wind
            )
            self.t += config.Schrittweite
            self.controller.regelschritt(
                self.quad,
                config.Schrittweite,
                self.sollgeschwindigkeit
            )
            self.drehlage = self.quaternion.quaternion2kardanwinkel(
                self.quad.zustand[3:7]
            )      
            self.speichereZustand()
            
            if (self.t > config.Episodenende):
                aktiv = 0

        return (
            self.t,
            self.quad.zustand[0],
            self.quad.zustand[1],
            self.quad.zustand[2],
            self.drehlage[2],
            -self.drehlage[1],
            self.drehlage[0]
        )


    def testModell(self):
        """Modell testen
        """
        aktiv = 1
        while (aktiv):

            aktion = self.vorhersage()
            # for _ in range(int(aktion[4] + 1)):
            #     self.quad.update(
            #         self.t, 
            #         aktion_final, 
            #         self.wind
            #     )
            #     self.t += config.Schrittweite

            # aktion_new = [random.randint(100, 700), random.randint(100, 700), random.randint(100, 700), random.randint(100, 700)]

            for _ in range(1):
                self.quad.update(
                    self.t, 
                    250 * aktion + 350, 
                    self.wind
                )
                self.t += config.Schrittweite

                self.speichereZustand()

                if (self.t > config.Episodenende):
                    aktiv = 0

        return (
            self.t,
            self.quad.zustand[0],
            self.quad.zustand[1],
            self.quad.zustand[2],
            self.drehlage[2],
            -self.drehlage[1],
            self.drehlage[0]
        )

    def speichereZustand(self):
        """Zustand abspeichern für die Visualisierung
        """
        self.t_all.append(self.t)
        self.x_soll.append(self.x_soll[-1] +
            config.Schrittweite * self.sollgeschwindigkeit[0]
        )
        self.y_soll.append(self.y_soll[-1] +
            config.Schrittweite * self.sollgeschwindigkeit[1]
        )
        self.z_soll.append(self.z_soll[-1] +
            config.Schrittweite * self.sollgeschwindigkeit[2]
        )
        self.geschwindigkeitssoll_x.append(self.sollgeschwindigkeit[0])
        self.geschwindigkeitssoll_y.append(self.sollgeschwindigkeit[1])
        self.geschwindigkeitssoll_z.append(self.sollgeschwindigkeit[2])
        self.geschwindigkeitsvektor_x.append(self.quad.zustand[7])
        self.geschwindigkeitsvektor_y.append(self.quad.zustand[8])
        self.geschwindigkeitsvektor_z.append(self.quad.zustand[9])
        self.x_all.append(self.quad.zustand[0])
        self.y_all.append(self.quad.zustand[1])
        self.z_all.append(self.quad.zustand[2])
        self.omega1_all.append(self.drehlage[0])
        self.omega2_all.append(-self.drehlage[1])
        self.omega3_all.append(self.drehlage[2])

    def render(self):
        """Rendern
        """
        # Create two subplots and unpack the output array immediately
        f, (ax1, ax2, ax3) = plt.subplots(
            1, 
            3, 
            sharex = True
        )

        f, (ax4, ax5, ax6) = plt.subplots(
            1, 
            3, 
            sharex = True,
            subplot_kw={'projection': 'polar'}
        )

        f, (ax7, ax8, ax9) = plt.subplots(
            1, 
            3, 
            sharex = True
        )

        ax1.plot(self.x_all)
        ax1.plot(self.x_soll, 'g')
        ax1.set_title('Norden/m')
        ax1.set_xlabel('t/ms')

        ax2.plot(self.y_all) 
        ax2.plot(self.y_soll, 'g')
        ax2.set_title('Osten/m')
        ax2.set_xlabel('t/ms')

        ax3.plot(self.z_all)
        ax3.plot(self.z_soll, 'g')
        ax3.set_title('Unten/m') 
        ax3.set_xlabel('t/ms')

        ax4.plot(self.omega1_all, self.t_all)
        ax4.set_title('Rollen')
        ax4.set_xlabel('t/s')

        ax5.plot(self.omega2_all, self.t_all)
        ax5.set_title('Nicken')
        ax5.set_xlabel('t/s')

        ax6.plot(self.omega3_all, self.t_all)
        ax6.set_title('Gieren')
        ax6.set_xlabel('t/s')

        ax7.plot(self.geschwindigkeitsvektor_x)
        ax7.plot(self.geschwindigkeitssoll_x, 'g')
        ax7.set_title('Norden (m/s)')
        ax7.set_xlabel('t/ms')

        ax8.plot(self.geschwindigkeitsvektor_y)
        ax8.plot(self.geschwindigkeitssoll_y, 'g')
        ax8.set_title('Osten (m/s)')
        ax8.set_xlabel('t/ms')

        ax9.plot(self.geschwindigkeitsvektor_z)
        ax9.plot(self.geschwindigkeitssoll_z, 'g')
        ax9.set_title('Unten (m/s)')
        ax9.set_xlabel('t/ms')

        plt.show()

    def simulation(self):
        """
        """
        start_time = time.time()

        # Simulation Setup
        Ti = 0
        Tf = 20
        ifsave = 0

        quad = Quadcopter()
        controller = Regler(quad)
        wind = Wind('None', 2.0, 90, -15)

        controller.regelschritt(
            quad, 
            config.Schrittweite
        )
        
        # Initialize Result Matrixes
        numTimeStep = int(Tf / config.Schrittweite + 1)

        t_all = numpy.zeros(numTimeStep)
        s_all = numpy.zeros([numTimeStep, len(quad.zustand)])
        pos_all = numpy.zeros(
            [numTimeStep, len(quad.zustand[0:3])]
        )
        vel_all = numpy.zeros(
            [numTimeStep, len(quad.zustand[7:10])]
        )
        quat_all = numpy.zeros(
            [numTimeStep, len(quad.zustand[3:7])]
        )
        omega_all = numpy.zeros(
            [numTimeStep, len(quad.zustand[10:13])]
        )
        euler_all = numpy.zeros(
            [numTimeStep, len(quad.euler)]
        )
        sDes_calc_all = numpy.zeros(
            [numTimeStep, len(self.sollgeschwindigkeit)]
        )
        w_cmd_all = numpy.zeros(
            [numTimeStep, len(controller.motorbefehle)]
        )
        wMotor_all = numpy.zeros(
            [numTimeStep, len(quad.wMotor)]
        )
        thr_all = numpy.zeros([numTimeStep, len(quad.thr)])
        tor_all = numpy.zeros([numTimeStep, len(quad.tor)])

        t_all[0]            = Ti
        s_all[0,:]          = quad.zustand
        pos_all[0,:]        = quad.zustand[0:3]
        vel_all[0,:]        = quad.zustand[7:10]
        quat_all[0,:]       = quad.zustand[3:7]
        omega_all[0,:]      = quad.zustand[10:13]
        euler_all[0,:]      = quad.euler
        sDes_calc_all[0,:]  = self.sollgeschwindigkeit
        w_cmd_all[0,:]      = controller.motorbefehle
        wMotor_all[0,:]     = quad.wMotor
        thr_all[0,:]        = quad.thr
        tor_all[0,:]        = quad.tor

        t = Ti
        i = 1

        reward = 0

        while round(t,3) < Tf:
            
            t = self.quad_sim(
                t, 
                quad, 
                controller, 
                wind
            )
            
            # print("{:.3f}".format(t))
            t_all[i]             = t
            s_all[i,:]           = quad.zustand
            pos_all[i,:]         = quad.zustand[0:3]
            vel_all[i,:]         = quad.zustand[7:10]
            quat_all[i,:]        = quad.zustand[3:7]
            omega_all[i,:]       = quad.zustand[10:13]
            euler_all[i,:]       = quad.euler
            sDes_calc_all[i,:]   = self.sollgeschwindigkeit
            w_cmd_all[i,:]       = controller.motorbefehle
            wMotor_all[i,:]      = quad.wMotor
            thr_all[i,:]         = quad.thr
            tor_all[i,:]         = quad.tor
            
            i += 1

            reward = reward + (

            )
        
        end_time = time.time()
        print(
            "Simulated {:.2f}s in {:.6f}s.".format(
                t, 
                end_time - start_time
            )
        )

        # utils.fullprint(sDes_traj_all[:,3:6])
        utils.makeFigures( 
            t_all, 
            pos_all, 
            vel_all, 
            quat_all, 
            omega_all, 
            euler_all, 
            w_cmd_all, 
            wMotor_all, 
            thr_all, 
            tor_all, 
            sDes_calc_all
        )
        ani = utils.sameAxisAnimation(
            t_all,
            pos_all, 
            quat_all, 
            config.Schrittweite,
            ifsave
        )
        plt.show()

    def quad_sim(
        self, 
        t, 
        quad, 
        controller, 
        wind):
        """
        """
    
        # Dynamics (using last timestep's commands)
        quad.update(
            t, 
            config.Schrittweite, 
            controller.motorbefehle, 
            wind
        )
        t += config.Schrittweite      

        # Generate Commands (for next iteration)
        controller.controller(
            quad, 
            config.Schrittweite,
            self.sollgeschwindigkeit
        )

        return t

if __name__ == '__main__':
    testbench = Testbench()
    # testbench.simulation()
    modeltyp = config.Ordnername[1]
    if modeltyp == "P":
        testbench.testPID()
    elif modeltyp == "M":
        testbench.testModell()
    else:
        print("Warnung! Invalider Reglertyp.")
    testbench.render()

