"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

"""
author: Tim Leonard Straube
email: hi@optimalpi.de
"""

import numpy
import config as config
from quaternion import Quaternion
from wind import Wind
from numpy import sin
from numpy import cos
from numpy import sign
from numpy.linalg import inv
from numpy.linalg import norm
from scipy.integrate import ode
from quadcontroller import Regler

deg2rad = numpy.pi / 180.0

class Quadcopter():
    def __init__(self):
        """Quadcopter including parameter and dynamics
        """
        super(Quadcopter, self).__init__()
        # total reward
        self.total_reward = 0
        # mass (kg)
        self.mass = 1
        # gravity (m/s^2)
        self.g = 9.81
        # arm length (m)
        self.dxm = 0.36      
        # arm length (m)
        self.dym = 0.36      
        # Motorhöhe über dem Schwerpunkt in m
        self.motorhöhe = 0.05  
        # Trägheitstensor (kg*m^2)
        self.IB  = numpy.array([
            [0.0123, 0,      0     ],
            [0,      0.0123, 0     ],
            [0,      0,      0.0224]
        ]) 
        # Rotor moment of inertia (kg * m^2)
        self.invI = inv(self.IB)
        self.IRzz = 2.7e-5

        # Include integral gains in linear velocity control
        self.useIntergral = bool(False)    
        # Interpolate Yaw setpoints in waypoint trajectory
        # params["interpYaw"] = bool(False)       

        # 
        self.Cd = 0.1
        # thrust coeff (N/(rad/s)^2)  (1.18e-7 N/RPM^2)
        self.kTh = 1.076e-5 
        # torque coeff (Nm/(rad/s)^2)  (1.79e-9 Nm/RPM^2)
        self.kTo = 1.632e-7 

        # Motor 1 is front left, then clockwise numbering.
        # A mixer like this one allows to find the exact RPM of each motor 
        # given a desired thrust and desired moments.
        # Inspiration for this mixer (or coefficient matrix) and how it is used 
        # https://link.springer.com/article/10.1007/s13369-017-2433-2 (https://sci-hub.tw/10.1007/s13369-017-2433-2)
        self.mixerFM = numpy.array([
            [             self.kTh,             self.kTh,   
                          self.kTh,             self.kTh],
            [ self.dym *  self.kTh, -self.dym * self.kTh, 
             -self.dym *  self.kTh,  self.dym * self.kTh],
            [ self.dxm *  self.kTh,  self.dxm * self.kTh,
             -self.dxm *  self.kTh, -self.dxm * self.kTh],
            [            -self.kTo,             self.kTo, 
                         -self.kTo,             self.kTo]
        ])

        self.mixerFMinv = inv(self.mixerFM)
        # Minimum total thrust
        self.minThr = 0.4    
        # Maximum total thrust
        self.maxThr = 4 * 9.18 * 4   
        # Minimum motor rotation speed (rad/s)
        self.minWmotor = 0       
        # Maximum motor rotation speed (rad/s)
        self.maxWmotor = 1000      
        # Value for second order system for Motor dynamics
        self.tau = 0.015    
        # Value for second order system for Motor dynamics
        self.kp = 1.0  
        # Value for second order system for Motor dynamics  
        self.dämpfung = 1.0  
        # w (rad/s) = cmd * c1 + c0 (cmd in %)
        self.motorc1 = 8.49  
        self.motorc0 = 74.7
        self.motordeadband = 1   
        # params["ifexpo"] = bool(False)
        # if params["ifexpo"]:
        #     params["maxCmd"] = 100      
        # # cmd (%) min and max
        #     params["minCmd"] = 0.01
        # else:
        #     params["maxCmd"] = 100
        #     params["minCmd"] = 1

        # Command for initial stable hover
        ini_hover = self.initialerMotorbefehl()

        # Feed-Forward Befehl fürs Scheben
        self.hoverfeedforward = ini_hover[0]   
        # Motordrehzahl fürs Schweben
        self.w_hover = ini_hover[1]    
        # Motordrehmoment fürs Schweben
        self.thr_hover = ini_hover[2]   
        self.thr = numpy.ones(4) * ini_hover[2]
        self.tor = numpy.ones(4) * ini_hover[3]

        self.quaternion = Quaternion()

        self.reset()

    def initialerMotorbefehl(self):
        """Berechnung des initalen Motorbefehls aus der Quadcoptermasse, der Graviation und Motorparametern
        """
        # w = cmd * c1 + c0 and m * g / 4 = kTh * w ^ 2 and torque = kTo * w ^ 2
        thr_hover = self.mass * self.g / 4.0
        w_hover = numpy.sqrt(thr_hover / self.kTh)
        tor_hover = self.kTo * w_hover * w_hover
        cmd_hover = (w_hover - self.motorc0) / self.motorc1
        return [cmd_hover, w_hover, thr_hover, tor_hover]

    def reset(self):
        """Environment reset
        """
        self.t = config.episode_start_time
        self.Ti = config.episode_start_time
        self.Ts = config.step_size

        # Agent
        self.schritt = 0

        # [rad]
        yaw = 0 # 2 * random.random() - 1
        # [rad]
        pitch = 0 # 2 * random.random() - 1
        # [rad] 
        roll = 0 # 2 * random.random() - 1

        quat = self.quaternion.kardanwinkel2quaternion(
            yaw,
            pitch, 
            roll
        )

        # Hovering motor acc
        self.wdot_hover = 0.0 

        self.sollposition = [0, 0, 0]

        # Sollgeschwindigkeit
        self.velocity_set = numpy.array([
            1,
            0,
            0
        ])

        # Zustandsvektorinitialisierung
        self.state = numpy.zeros(21)

        # Norden [m]
        self.state[0] = 0
        # Osten [m]
        self.state[1] = 0
        # Unten [m]
        self.state[2] = 0

        # Quaternion Realteil
        self.state[3] = quat[0]
        # Quaternion erster Imaginärteil 
        self.state[4] = quat[1]  
        # Quaternion zweiter Imaginärteil
        self.state[5] = quat[2]  
        # Quaternion dritter Imaginärteil  
        self.state[6] = quat[3]  
        # Geschwindigkeit nach Norden [m/s]
        self.state[7] = 0.0    
        # Geschwindigkeit nach Osten [m/s]
        self.state[8] = 0.0    
        # Geschindigkeit nach Unten [m/s]
        self.state[9] = 0.0    
        # Rollen [rad/s]
        self.state[10] = 0.0    
        # Nicken [rad/s]
        self.state[11] = 0.0    
        # Gieren [rad/s]
        self.state[12] = 0.0
        # Motor 1 Drehgeschwindigkeit [rad/s]
        self.state[13] = self.w_hover
        # Motor 1 Beschleunigung [rad/s^2]
        self.state[14] = self.wdot_hover
        # Motor 2 Drehgeschwindigkeit [rad/s]
        self.state[15] = self.w_hover
        # Motor 2 Beschleunigung [rad/s^2]
        self.state[16] = self.wdot_hover
        # Motor 3 Drehgeschwindigkeit [rad/s]
        self.state[17] = self.w_hover
        # Motor 3 Beschleunigung [rad/s^2]
        self.state[18] = self.wdot_hover
        # Motor 4 Drehgeschwindigkeit [rad/s]
        self.state[19] = self.w_hover
        # Motor 4 Beschleunigung [rad/s^2]
        self.state[20] = self.wdot_hover

        # Initiale Motorgeschwindigkeit
        self.wMotor = numpy.array([
            self.state[13],
            self.state[15],
            self.state[17],
            self.state[19]
        ])
        self.vel_dot = numpy.zeros(3)
        self.omega_dot = numpy.zeros(3)
        self.acceleration = numpy.zeros(3)

        self.extended_state()
        self.forces()

        self.controller = Regler(self)

        self.controller.regelschritt(
            self, 
            config.step_size,
            self.velocity_set
        )

        # wind model
        self.wind = Wind('NONE', 0.0, 0, -15)

        self.integrator = ode(
            self.state_dot
        ).set_integrator(
            'dopri5',
            first_step='0.00005',
            atol='10e-6',
            rtol='10e-6'
        )
        self.integrator.set_initial_value(
            self.state, 
            config.episode_start_time
        )

        self.drehlage = self.quaternion.quaternion2cardan(
            self.state[3:7]
        )   

    def extended_state(self):
        """Zustandumrechnung 
        """
        # Drehlageumrechnung zu einer Rotationsmatrix
        self.dcm = self.quaternion.quat2Dcm(
            self.state[3:7]
        )

        # Drehlageumrechnung zu Kardanwinkeln
        YPR = self.quaternion.quaternion2cardan(
            self.state[3:7]
        )
        # flip YPR so that euler state = phi, theta, psi
        self.euler = YPR[::-1] 
        self.psi   = YPR[0]
        self.theta = YPR[1]
        self.phi   = YPR[2]
    
    def forces(self):
        """
        """
        # Schubberechnung aus der Motordrehzahl
        self.thr = self.kTh * self.wMotor * self.wMotor
        # Drehmomentberechnung aus der Motordrehzahl
        self.tor = self.kTo * self.wMotor * self.wMotor

    def state_dot(self, t, s, motorbefehle, wind):
        """
        """
    
        # Importieren des Zustandsvektors 
        norden = s[0]
        osten = s[1]
        unten = s[2]
        q_w = s[3]
        q_x = s[4]
        q_y = s[5]
        q_z = s[6]
        xdot = s[7]
        ydot = s[8]
        zdot = s[9]
        # Rollen
        p = s[10]
        # Nicken
        q = s[11]
        # Gieren
        r = s[12]
        wM1 = s[13]
        wdotM1 = s[14]
        wM2 = s[15]
        wdotM2 = s[16]
        wM3 = s[17]
        wdotM3 = s[18]
        wM4 = s[19]
        wdotM4 = s[20]

        # Motor Dynamics and Rotor forces (Second Order System: https://apmonitor.com/pdc/index.php/Main/SecondOrderSystems)
        
        wddotM1 = (
            -2.0 * self.dämpfung * self.tau * wdotM1 - 
            wM1 + 
            self.kp * motorbefehle[0]
        ) / (self.tau ** 2)
        wddotM2 = (
            -2.0 * self.dämpfung * self.tau * wdotM2 - 
            wM2 + 
            self.kp * motorbefehle[1]
        ) / (self.tau ** 2)
        wddotM3 = (
            -2.0 * self.dämpfung * self.tau * wdotM3 - 
            wM3 + 
            self.kp * motorbefehle[2]
        ) / (self.tau ** 2)
        wddotM4 = (
            -2.0 * self.dämpfung * self.tau * wdotM4 - 
            wM4 + 
            self.kp * motorbefehle[3]
        ) / (self.tau ** 2)
    
        wMotor = numpy.array([wM1, wM2, wM3, wM4])
        wMotor = numpy.clip(
            wMotor, 
            self.minWmotor, 
            self.maxWmotor
        )
        thrust = self.kTh * wMotor * wMotor
        drehmoment = self.kTo * wMotor * wMotor
    
        ThrM1 = thrust[0]
        ThrM2 = thrust[1]
        ThrM3 = thrust[2]
        ThrM4 = thrust[3]
        TorM1 = drehmoment[0]
        TorM2 = drehmoment[1]
        TorM3 = drehmoment[2]
        TorM4 = drehmoment[3]

        # Windmodel
        [velW, qW1, qW2] = wind.randomWind(t)
        # velW = 0

        # velW = 5          # m/s
        # qW1 = 0 * deg2rad    # Wind heading
        # qW2 = 60 * deg2rad     # Wind elevation (positive = upwards wind in NED, positive = downwards wind in ENU)
    
        # Inertialtensor
        IBxx = self.IB[0, 0]
        IByy = self.IB[1, 1]
        IBzz = self.IB[2, 2]

        # State Derivatives (from PyDy) This is already the analytically solved vector of MM*x = RHS
        sdotn = numpy.array([
            [xdot],
            [ydot],
            [zdot],
            [-0.5 * p * q_x - 0.5 * q * q_y - 0.5 * q_z * r],
            [ 0.5 * p * q_w - 0.5 * q * q_z + 0.5 * q_y * r],
            [ 0.5 * p * q_z + 0.5 * q * q_w - 0.5 * q_x * r],
            [-0.5 * p * q_y + 0.5 * q * q_x + 0.5 * q_w * r],
            [( self.Cd * sign(velW * cos(qW1) * cos(qW2) - xdot) * (velW * cos(qW1) * cos(qW2) - xdot) ** 2 - 2 * (q_w * q_y + q_x * q_z) * (ThrM1 + ThrM2 + ThrM3 + ThrM4)) / self.mass],
            [( self.Cd * sign(velW * sin(qW1) * cos(qW2) - ydot) * (velW * sin(qW1) * cos(qW2) - ydot) ** 2 + 2 * (q_w * q_x - q_y * q_z) * (ThrM1 + ThrM2 + ThrM3 + ThrM4)) / self.mass],
            [(-self.Cd * sign(velW * sin(qW2) + zdot) * (velW * sin(qW2) + zdot) ** 2 - (ThrM1 + ThrM2 + ThrM3 + ThrM4) * (q_w ** 2 - q_x ** 2 - q_y ** 2 + q_z ** 2) + self.g * self.mass) / self.mass],
            [((IByy - IBzz) * q * r + (ThrM1 - ThrM2 - ThrM3 + ThrM4) * self.dym) / IBxx],
            [((IBzz - IBxx) * p * r + (ThrM1 + ThrM2 - ThrM3 - ThrM4) * self.dxm) / IByy], 
            [((IBxx - IByy) * p * q - TorM1 + TorM2 - TorM3 + TorM4) / IBzz]])
    
        # State Derivative Vector
        sdot = numpy.zeros([21])
        sdot[0] = sdotn[0]
        sdot[1] = sdotn[1]
        sdot[2] = sdotn[2]
        sdot[3] = sdotn[3]
        sdot[4] = sdotn[4]
        sdot[5] = sdotn[5]
        sdot[6] = sdotn[6]
        sdot[7] = sdotn[7]
        sdot[8] = sdotn[8]
        sdot[9] = sdotn[9]
        sdot[10] = sdotn[10]
        sdot[11] = sdotn[11]
        sdot[12] = sdotn[12]
        sdot[13] = wdotM1
        sdot[14] = wddotM1
        sdot[15] = wdotM2
        sdot[16] = wddotM2
        sdot[17] = wdotM3
        sdot[18] = wddotM3
        sdot[19] = wdotM4
        sdot[20] = wddotM4

        self.acceleration = sdot[7:10]

        # self.state = sdot

        return sdot

    def update(self, t, motorbefehle, wind):
        """This method ensures that the quadcopter's state is continuously updated based on the given inputs, allowing for accurate simulation or control of its behavior.
        """
        geschwindigkeit_t_minus_1 = self.state[7:10]
        omega_t_minus_1 = self.state[10:13]

        self.integrator.set_f_params(motorbefehle, wind)
        self.state = self.integrator.integrate(
            t, 
            t + config.step_size
        )

        self.pos = self.state[0:3]
        self.quat = self.state[3:7]
        self.geschwindigkeit = self.state[7:10]
        self.omega = self.state[10:13]
        self.wMotor = numpy.array([
            self.state[13], 
            self.state[15], 
            self.state[17],
            self.state[19]
        ])

        self.vel_dot = (
            self.geschwindigkeit - geschwindigkeit_t_minus_1
        ) / config.step_size
        self.omega_dot = (
            self.omega - omega_t_minus_1
        ) / config.step_size

        self.extended_state()
        self.forces()

    def belohnung(self):
        """Method for calculating the reward based on the quadcopter state and reward weighting.
        """
        error_velocity = (
            self.state[7:10] -
            self.velocity_set
        )
        drehgeschwindigkeitfehler = (
            self.state[10:13] - 
            [0, 0, 0]
        )
        attitude = self.quaternion.quaternion2cardan(
            self.state[3:7]
        )      
        error_attitude = numpy.abs(attitude[0:3])

        return ( 
            -config.reward_weights[1] *
            (numpy.sum(numpy.abs(error_velocity))) +
            config.reward_weights[2] /
            (1 + numpy.sum(drehgeschwindigkeitfehler ** 2)) +
            config.reward_weights[3] / 
            (1 + numpy.sum(error_attitude ** 2))
        )