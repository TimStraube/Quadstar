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
email: hi@optimalpi.de
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

NORTH = 0
EAST = 1
DOWN = 2
ROLL = 0
PITCH = 1
YAW = 2

class ControllerPID():
    def __init__(self, quadcopter):
        """PID-Quadcopterregler
        :param quadcopter: Quadcopter
        :return: 
        """
        self.step_current = 0
        self.deg2rad = pi / 180.0

        # NED
        self.vel_P_gain = np.array([5.0, 5.0, 4.0])
        self.vel_D_gain = np.array([0.5, 0.5, 0.5])
        self.vel_I_gain = np.array([5.0, 5.0, 5.0])

        # Attitude P gains
        self.attitute_p_gain = np.array([
            8.0, 
            8.0, 
            1.5
        ])
        self.rate_P_gain = np.array([
            1.5, 
            1.5, 
            1.0
        ])
        self.rate_D_gain = np.array([
            0.04, 
            0.04, 
            0.1
        ])

        # Max velocity
        self.uMax = 5.0
        self.vMax = 5.0
        self.wMax = 5.0

        self.velocity_max = np.array([
            self.uMax, 
            self.vMax, 
            self.wMax
        ])
        self.velocity_max_all = 5.0

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

        # Initalization of motor commands
        self.motor_commands = np.ones(4) * quadcopter.w_hover
        self.thrust_integral = np.zeros(3)

        # if (yawType == 0):
        #     self.attitute_p_gain[2] = 0

        self.setYawWeight()
        self.position_set = np.zeros(3)
        self.velocity_set = np.zeros(3)
        self.thrust_set = np.zeros(3)
        self.eul_sp = np.zeros(3)
        self.pqr_sp = np.zeros(3)
        self.yawfeedforward = np.zeros(3)
        self.gain = 0

        self.quaternion = Quaternion()
    
    def controller_step(
        self, 
        quadcopter, 
        step_size, 
        velocity_set):

        """Calculate the motor commands from the velocity setpoint
        :param quadcopter: Quadcopter
        :param step_size: step size
        :param velocity_set: velocity setpoint
        :return: 
        """
        self.velocity_set[:] = velocity_set
        self.thrust_set[:] = [0, 0, 0]

        self.step_current += 1

        # self.saturate_velocity()
        self.controller_down(quadcopter)
        self.controller_north_east(quadcopter, step_size)
        self.thrust2attitude()
        self.controller_attitude(quadcopter)
        self.rate_control(quadcopter)
        self.rate2cmd(quadcopter)

        
    def saturate_velocity(self):
        # Saturate Velocity Setpoint
        # Either saturate each velocity axis separately, or total velocity (prefered)
        if (self.saturateVel_separetely):
            self.velocity_set = np.clip(
                self.velocity_set, 
                -self.velocity_max, 
                self.velocity_max
            )
        else:
            totalVel_sp = norm(self.velocity_set)
            if (totalVel_sp > self.velocity_max_all):
                self.velocity_set = (
                    self.velocity_set / 
                    totalVel_sp * 
                    self.velocity_max_all
                )

    # Test
    def controller_down(self, quadcopter):
        """Geschwindigkeitsregler für die Unten-Achse
        """
        # Z Velocity Control (Thrust in D-direction)
        # Hover thrust (m*g) is sent as a Feed-Forward term, in order to allow hover when the position and velocity error are null
        velocity_error_down = (
            self.velocity_set[2] - 
            quadcopter.state[9]
        )
        thrust_set_down = (
            self.vel_P_gain[DOWN] * velocity_error_down - 
            self.vel_D_gain[DOWN] * quadcopter.acceleration[DOWN] + 
            (quadcopter.velocity_set[2] - quadcopter.mass) * 
            quadcopter.g + 
            self.thrust_integral[2]
        )
        
        # Get thrust limits
        # The Thrust limits are negated and swapped due to NED-frame
        uMax = -quadcopter.minThr
        uMin = -quadcopter.maxThr

        # Apply Anti-Windup in D-direction
        stop_int_D = (
            (thrust_set_down >= uMax and velocity_error_down >= 0.0) or 
            (thrust_set_down <= uMin and velocity_error_down <= 0.0)
        )

        # Calculate integral part
        # if not (stop_int_D):
        #     self.thrust_integral[2] += (
        #         self.vel_I_gain[2] * velocity_error_down * step_size * quadcopter.useIntergral
        #     )
        #     # Limit thrust integral
        #     self.thrust_integral[2] = (
        #         min(abs(self.thrust_integral[2]), quadcopter.maxThr) * 
        #         np.sign(self.thrust_integral[2])
        #     )

        # Saturate thrust setpoint in D-direction
        self.thrust_set[2] = np.clip(
            thrust_set_down, 
            uMin, 
            uMax
        )
    
    def controller_north_east(self, quadcopter, step_size):
        """
        Geschwindigkeitsregler für die Geschwindigkeit auf der Nord- und Ost-Achse
        """
        # XY Velocity Control (Thrust in NE-direction)
        velocity_error_north_east = (
            self.velocity_set[0:2] - quadcopter.state[7:9]
        )
        thrust_set_north_east = (
            self.vel_P_gain[0:2] * velocity_error_north_east - 
            self.vel_D_gain[0:2] * quadcopter.acceleration[0:2] + 
            self.thrust_integral[0:2]
        )

        # Max allowed thrust in NE based on tilt and excess thrust
        thrust_max_xy_tilt = (
            abs(self.thrust_set[DOWN]) * 
            np.tan(self.tiltMax)
        )
        thrust_max_xy = sqrt(
            quadcopter.maxThr ** 2 - 
            self.thrust_set[DOWN] ** 2
        )
        thrust_max_xy = min(
            thrust_max_xy, 
            thrust_max_xy_tilt
        )

        # Saturate thrust in NE-direction
        self.thrust_set[0:2] = thrust_set_north_east
        if (
            np.dot(
                self.thrust_set[0:2].T, 
                self.thrust_set[0:2]) > 
            thrust_max_xy ** 2):

            mag = norm(self.thrust_set[0:2])
            self.thrust_set[0:2] = (
                thrust_set_north_east / mag * thrust_max_xy
            )
        
        # Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
        # see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
        arw_gain = 2.0 / self.vel_P_gain[0:2]
        vel_err_lim = (
            velocity_error_north_east - 
            (thrust_set_north_east - self.thrust_set[0:2]) * arw_gain
        )
        self.thrust_integral[0:2] += (
            self.vel_I_gain[0:2] * 
            vel_err_lim * 
            step_size * 
            quadcopter.useIntergral
        )
    
    def thrust2attitude(self):
        # Create Full Desired Quaternion Based on Thrust Setpoint and Desired Yaw Angle
        yaw_set = self.eul_sp[DOWN]

        # Desired body_z axis direction
        body_z = -self.quaternion.vectNormalize(
            self.thrust_set
        )
        
        # Vector of desired Yaw direction in XY plane, rotated by pi/2 (fake body_y axis)
        y_C = np.array([
            -sin(yaw_set), 
            cos(yaw_set), 
            0.0
        ])
        
        # Desired body_x axis direction
        body_x = np.cross(y_C, body_z)
        body_x = self.quaternion.vectNormalize(body_x)
        
        # Desired body_y axis direction
        body_y = np.cross(body_z, body_x)

        # Sollrotationsmatrix
        R_set = np.array([body_x, body_y, body_z]).T

        # Full desired quaternion (full because it considers the desired Yaw angle)
        self.qd_full = self.quaternion.rotationsmatrix2quaternion(R_set)
        
    def controller_attitude(self, quadcopter):
        # Current thrust orientation e_z and desired thrust orientation e_z_d
        e_z = quadcopter.dcm[:, 2]
        e_z_d = -self.quaternion.vectNormalize(
            self.thrust_set
        )

        # Quaternionfehler zwei Vektoren
        quaternion_error_without_yaw = np.zeros(4)
        quaternion_error_without_yaw[0] = np.dot(
            e_z, e_z_d
        ) + sqrt(
            norm(e_z) ** 2 * 
            norm(e_z_d) ** 2
        )
        quaternion_error_without_yaw[1:4] = np.cross(
            e_z, 
            e_z_d
        )
        quaternion_error_without_yaw = self.quaternion.vectNormalize(
            quaternion_error_without_yaw
        )
        
        # Reduzierte Sollquaternion ohne den Yaw Winkel 
        self.quaternion_set_without_yaw = self.quaternion.quatMultiply(
            quaternion_error_without_yaw, 
            quadcopter.state[3:7]
        )

        # Mixed desired quaternion (between reduced and full) and resulting desired quaternion qd
        q_mix = self.quaternion.quatMultiply(
            self.quaternion.inverse(
                self.quaternion_set_without_yaw
            ), 
            self.qd_full
        )
        q_mix = q_mix * np.sign(q_mix[0])
        q_mix[0] = np.clip(q_mix[0], -1.0, 1.0)
        q_mix[3] = np.clip(q_mix[3], -1.0, 1.0)
        self.qd = self.quaternion.quatMultiply(
            self.quaternion_set_without_yaw, 
            np.array([
                cos(self.yaw_w * np.arccos(q_mix[0])), 
                0, 
                0, 
                sin(self.yaw_w * np.arcsin(q_mix[3]))
            ])
        )

        # Resulting error quaternion
        self.quadcopterternion_error = self.quaternion.quatMultiply(
            self.quaternion.inverse(quadcopter.state[3:7]), 
            self.qd
        )

        # Create rate setpoint from quaternion error
        self.rate_sp = ((
            2.0 * 
            np.sign(self.quadcopterternion_error[0]) * 
            self.quadcopterternion_error[1:4]) * 
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
            self.quaternion.inverse(quadcopter.state[3:7])
        )[:, 2] * self.yawfeedforward

        # Limit rate setpoint
        self.rate_sp = np.clip(
            self.rate_sp, 
            -self.rateMax, 
            self.rateMax
        )

    def rate_control(self, quadcopter):
        # Rate Control
        # Be sure it is right sign for the D part
        rate_error = self.rate_sp - quadcopter.state[10:13]
        self.rate_set = (
            self.rate_P_gain * rate_error - 
            self.rate_D_gain * quadcopter.omega_dot
        )    

    def setYawWeight(self):
        """Calculate weights of the yaw gain
        """
        roll_pitch_gain = (
            0.5 * (
                self.attitute_p_gain[ROLL] + 
                self.attitute_p_gain[PITCH]
            )
        )
        self.yaw_w = np.clip(
            self.attitute_p_gain[YAW] / roll_pitch_gain, 
            0.0, 
            1.0
        )
        self.attitute_p_gain[YAW] = roll_pitch_gain

    def rate2cmd(self, quadcopter):
        """Mix desired rates to motor commands
        """
        # Mixer
        t = np.array([
            norm(self.thrust_set), 
            self.rate_set[0], 
            self.rate_set[1], 
            self.rate_set[2]
        ])
        self.motor_commands = np.sqrt(np.clip(
            np.dot(quadcopter.mixerFMinv, t),
            quadcopter.minWmotor ** 2, 
            quadcopter.maxWmotor ** 2
        ))