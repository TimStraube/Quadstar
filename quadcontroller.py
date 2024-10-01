# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

# Position and Velocity Control based on https://github.com/PX4/Firmware/blob/master/src/modules/mc_pos_control/PositionControl.cpp
# Desired Thrust to Desired Attitude based on https://github.com/PX4/Firmware/blob/master/src/modules/mc_pos_control/Utility/ControlMath.cpp
# Attitude Control based on https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp
# and https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf
# Rate Control based on https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control/mc_att_control_main.cpp

"""
author: Tim Leonard Straube
organization: HTWG Konstanz
email: ti741str@htwg-konstanz.de
comment: PID-Regler
"""

import numpy as np
import config as config
from numpy import pi
from numpy import sin
from numpy import cos
from numpy import tan
from numpy import sqrt
from numpy.linalg import norm
from quaternion import Quaternion

NORDEN = 0
OSTEN = 1
UNTEN = 2
ROLLEN = 0
NICKEN = 1
GIEREN = 2

class Regler():
    def __init__(self, quad):
        """PID-Quadcopterregler
        :param quad: Quadcopter
        :return: 
        """
        self.schritt = 0
        self.deg2rad = pi / 180.0

        self.pos_P_gain = np.array([1.0, 1.0, 1.0])

        self.vel_P_gain = np.array([5.0, 5.0, 4.0])

        self.vel_D_gain = np.array([0.5, 0.5, 0.5])

        self.vel_I_gain = np.array([5.0, 5.0, 5.0])

        # Attitude P gains
        self.attitute_p_gain = np.array([
            8.0, 
            8.0, 
            1.5
        ])

        # Rate P-D gains
        self.Pp = 1.5
        self.Dp = 0.04
        self.Pq = self.Pp
        self.Dq = self.Dp
        self.Pr = 1.0
        self.Dr = 0.1

        self.rate_P_gain = np.array([
            self.Pp, 
            self.Pq, 
            self.Pr
        ])
        self.rate_D_gain = np.array([
            self.Dp, 
            self.Dq, 
            self.Dr
        ])

        # Maximale Geschwindigkeit
        self.uMax = 5.0
        self.vMax = 5.0
        self.wMax = 5.0

        self.velMax = np.array([
            self.uMax, 
            self.vMax, 
            self.wMax
        ])
        self.velMaxAll = 5.0

        self.saturateVel_separetely = False

        # Maximale Tiltung
        self.tiltMax = 50.0 * self.deg2rad

        # Max Rate
        self.pMax = 200.0 * self.deg2rad
        self.qMax = 200.0 * self.deg2rad
        self.rMax = 150.0 * self.deg2rad

        self.rateMax = np.array([
            self.pMax, 
            self.qMax, 
            self.rMax
        ])

        # Initalisierung der Motorbefehle
        self.motorbefehle = np.ones(4) * quad.w_hover
        self.schubintegral = np.zeros(3)

        # if (yawType == 0):
        #     self.attitute_p_gain[2] = 0

        self.setYawWeight()
        self.pos_sp = np.zeros(3)
        self.sollgeschwindigkeit = np.zeros(3)
        self.sollschub = np.zeros(3)
        self.eul_sp = np.zeros(3)
        self.pqr_sp = np.zeros(3)
        self.yawfeedforward = np.zeros(3)
        self.gain = 0

        self.quaternion = Quaternion()
    
    def regelschritt(self, quad, Ts, sollgeschwindigkeit):
        """ Berechnung der Motorbefehle aus dem Quadcopterzustandsvektor und der Sollgeschwindigkeit
        :param quad: Quadcopter
        :param Ts: Schrittweite
        :param sollgeschwindigkeit: Sollgeschwindigkeit
        :return: 
        """
        self.sollgeschwindigkeit[:] = sollgeschwindigkeit
        self.sollschub[:] = [0, 0, 0]

        self.schritt += 1

        # self.saturateVel()
        self.regler_dz(quad)
        self.regler_dxdy(quad, Ts)
        self.thrustToAttitude()
        self.regler_lage(quad)
        self.rate_control(quad)
        self.motorbefehleberechnen(quad)

        
    def saturateVel(self):
        # Saturate Velocity Setpoint
        # Either saturate each velocity axis separately, or total velocity (prefered)
        if (self.saturateVel_separetely):
            self.sollgeschwindigkeit = np.clip(
                self.sollgeschwindigkeit, 
                -self.velMax, 
                self.velMax
            )
        else:
            totalVel_sp = norm(self.sollgeschwindigkeit)
            if (totalVel_sp > self.velMaxAll):
                self.sollgeschwindigkeit = (
                    self.sollgeschwindigkeit / 
                    totalVel_sp * 
                    self.velMaxAll
                )

    # Test
    def regler_dz(self, quad):
        """
        Geschwindigkeitsregler für die Unten-Achse
        INPUT: vel_P_gain[UNTEN]
        """
        # Z Velocity Control (Thrust in D-direction)
        # Hover thrust (m*g) is sent as a Feed-Forward term, in order to allow hover when the position and velocity error are null
        geschwindigkeitsfehler_z = (
            self.sollgeschwindigkeit[2] - 
            quad.zustand[9]
        )
        sollschub_z = (
            self.vel_P_gain[UNTEN] * geschwindigkeitsfehler_z - 
            self.vel_D_gain[UNTEN] * quad.beschleunigung[2] + 
            (quad.sollgeschwindigkeit[2] - quad.quadcoptermasse) * 
            quad.gravitationskonstante + 
            self.schubintegral[2]
        )
        
        # Get thrust limits
        # The Thrust limits are negated and swapped due to NED-frame
        uMax = -quad.minThr
        uMin = -quad.maxThr

        # Apply Anti-Windup in D-direction
        stop_int_D = (
            (sollschub_z >= uMax and geschwindigkeitsfehler_z >= 0.0) or 
            (sollschub_z <= uMin and geschwindigkeitsfehler_z <= 0.0)
        )

        # Calculate integral part
        # if not (stop_int_D):
        #     self.schubintegral[2] += (
        #         self.vel_I_gain[2] * geschwindigkeitsfehler_z * Ts * quad.useIntergral
        #     )
        #     # Limit thrust integral
        #     self.schubintegral[2] = (
        #         min(abs(self.schubintegral[2]), quad.maxThr) * 
        #         np.sign(self.schubintegral[2])
        #     )

        # Saturate thrust setpoint in D-direction
        self.sollschub[2] = np.clip(sollschub_z, uMin, uMax)
    
    def regler_dxdy(self, quad, Ts):
        """
        Geschwindigkeitsregler für die Geschwindigkeit auf der Nord- und Ost-Achse
        """
        # XY Velocity Control (Thrust in NE-direction)
        geschwindigkeitsfehler_xy = self.sollgeschwindigkeit[0:2] - quad.zustand[7:9]
        thrust_xy_sp = (
            self.vel_P_gain[0:2] * geschwindigkeitsfehler_xy - 
            self.vel_D_gain[0:2] * quad.beschleunigung[0:2] + 
            self.schubintegral[0:2]
        )

        # Max allowed thrust in NE based on tilt and excess thrust
        thrust_max_xy_tilt = (
            abs(self.sollschub[UNTEN]) * 
            np.tan(self.tiltMax)
        )
        thrust_max_xy = sqrt(
            quad.maxThr ** 2 - 
            self.sollschub[UNTEN] ** 2
        )
        thrust_max_xy = min(
            thrust_max_xy, 
            thrust_max_xy_tilt
        )

        # Saturate thrust in NE-direction
        self.sollschub[0:2] = thrust_xy_sp
        if (
            np.dot(
                self.sollschub[0:2].T, 
                self.sollschub[0:2]) > 
            thrust_max_xy ** 2
            ):

            mag = norm(self.sollschub[0:2])
            self.sollschub[0:2] = (
                thrust_xy_sp / mag * thrust_max_xy
            )
        
        # Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
        # see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
        arw_gain = 2.0 / self.vel_P_gain[0:2]
        vel_err_lim = (
            geschwindigkeitsfehler_xy - 
            (thrust_xy_sp - self.sollschub[0:2]) * arw_gain
        )
        self.schubintegral[0:2] += (
            self.vel_I_gain[0:2] * 
            vel_err_lim * 
            Ts * 
            quad.useIntergral
        )
    
    def thrustToAttitude(self):
        # Create Full Desired Quaternion Based on Thrust Setpoint and Desired Yaw Angle
        sollgieren = self.eul_sp[UNTEN]

        # Desired body_z axis direction
        body_z = -self.quaternion.vectNormalize(
            self.sollschub
        )
        
        # Vector of desired Yaw direction in XY plane, rotated by pi/2 (fake body_y axis)
        y_C = np.array([
            -sin(sollgieren), 
            cos(sollgieren), 
            0.0
        ])
        
        # Desired body_x axis direction
        body_x = np.cross(y_C, body_z)
        body_x = self.quaternion.vectNormalize(body_x)
        
        # Desired body_y axis direction
        body_y = np.cross(body_z, body_x)

        # Sollrotationsmatrix
        R_sp = np.array([body_x, body_y, body_z]).T

        # Full desired quaternion (full because it considers the desired Yaw angle)
        self.qd_full = self.quaternion.rotationsmatrix2quaternion(R_sp)
        
    def regler_lage(self, quad):
        # Current thrust orientation e_z and desired thrust orientation e_z_d
        e_z = quad.dcm[:, 2]
        e_z_d = -self.quaternion.vectNormalize(
            self.sollschub
        )

        # Quaternionfehler zwei Vektoren
        fehlerquaternion_reduziert = np.zeros(4)
        fehlerquaternion_reduziert[0] = np.dot(
            e_z, e_z_d
        ) + sqrt(
            norm(e_z) ** 2 * 
            norm(e_z_d) ** 2
        )
        fehlerquaternion_reduziert[1:4] = np.cross(
            e_z, 
            e_z_d
        )
        fehlerquaternion_reduziert = self.quaternion.vectNormalize(
            fehlerquaternion_reduziert
        )
        
        # Reduzierte Sollquaternion ohne den Yaw Winkel 
        self.sollquaternion_reduziert = self.quaternion.quatMultiply(
            fehlerquaternion_reduziert, 
            quad.zustand[3:7]
        )

        # Mixed desired quaternion (between reduced and full) and resulting desired quaternion qd
        q_mix = self.quaternion.quatMultiply(
            self.quaternion.inverse(
                self.sollquaternion_reduziert
            ), 
            self.qd_full
        )
        q_mix = q_mix * np.sign(q_mix[0])
        q_mix[0] = np.clip(q_mix[0], -1.0, 1.0)
        q_mix[3] = np.clip(q_mix[3], -1.0, 1.0)
        self.qd = self.quaternion.quatMultiply(
            self.sollquaternion_reduziert, 
            np.array([
                cos(self.yaw_w * np.arccos(q_mix[0])), 
                0, 
                0, 
                sin(self.yaw_w * np.arcsin(q_mix[3]))
            ])
        )

        # Resulting error quaternion
        self.quadternion_error = self.quaternion.quatMultiply(
            self.quaternion.inverse(quad.zustand[3:7]), 
            self.qd
        )

        # Create rate setpoint from quaternion error
        self.rate_sp = ((
            2.0 * 
            np.sign(self.quadternion_error[0]) * 
            self.quadternion_error[1:4]) * 
            self.attitute_p_gain
        )
        
        # Limit yawFF
        self.yawfeedforward = np.clip(
            self.yawfeedforward, 
            -self.rateMax[2], 
            self.rateMax[2]
        )

        # Add Yaw rate feed-forward
        self.rate_sp += self.quaternion.quat2Dcm(
            self.quaternion.inverse(quad.zustand[3:7])
        )[:, 2] * self.yawfeedforward

        # Limit rate setpoint
        self.rate_sp = np.clip(
            self.rate_sp, 
            -self.rateMax, 
            self.rateMax
        )

    def rate_control(self, quad):
        # Rate Control
        rate_error = self.rate_sp - quad.zustand[10:13]
        self.drehratenstellwert = (
            self.rate_P_gain * rate_error - 
            self.rate_D_gain * quad.omega_dot
        )    # Be sure it is right sign for the D part

    def setYawWeight(self):
        """
        Berechnung des Gewichts vom Gierkontrollgain
        """
        roll_pitch_gain = (
            0.5 * (
                self.attitute_p_gain[ROLLEN] + 
                self.attitute_p_gain[NICKEN]
            )
        )
        self.yaw_w = np.clip(
            self.attitute_p_gain[GIEREN] / roll_pitch_gain, 
            0.0, 
            1.0
        )
        self.attitute_p_gain[GIEREN] = roll_pitch_gain

    def motorbefehleberechnen(self, quad):
        """
        """
        # Mixer
        t = np.array([
            norm(self.sollschub), 
            self.drehratenstellwert[0], 
            self.drehratenstellwert[1], 
            self.drehratenstellwert[2]
        ])
        self.motorbefehle = np.sqrt(np.clip(
            np.dot(quad.mixerFMinv, t),
            quad.minWmotor ** 2, 
            quad.maxWmotor ** 2
        ))