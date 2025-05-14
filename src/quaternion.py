"""
credit: John Bass
email: john.bobzwik@gmail.com
"""

"""
author: Tim Leonard Straube
email: hi@optimalpi.de
"""

import numpy

class Quaternion():
    def __init__(self) -> None:
        """Methods for modeling and transforming quaternions
        """
        pass

    def threeaxisrot(self, r11, r12, r21, r31, r32):
        """Rotation around three axes
        """
        r1 = numpy.arctan2(r11, r12)
        r2 = numpy.arcsin(r21)
        r3 = numpy.arctan2(r31, r32)

        return numpy.array([r1, r2, r3])

    def quaternion2cardan(self, q):
        """Transformation of quaternions to cardan angles
        params: q: quaternion [qw, qx, qy, qz]
        return: cardan: [yaw, pitch, roll]
        """
        qw = q[0]
        qx = q[1]
        qy = q[2]
        qz = q[3]
        
        kardanwinkel = self.threeaxisrot(
            2.0 * (qx * qy + qw * qz),
            qw ** 2 + qx ** 2 - qy ** 2 - qz ** 2,
            -2.0 * (qx * qz - qw * qy),
            2.0 * (qy * qz + qw * qx),
            qw ** 2 - qx ** 2 - qy ** 2 + qz ** 2
        )
        return kardanwinkel
    
    def quat2Dcm(self, q):
        """Quaternion to rotation matrix
        """
        dcm = numpy.zeros([3, 3])

        dcm[0, 0] = (
            q[0] ** 2 + 
            q[1] ** 2 - 
            q[2] ** 2 - 
            q[3] ** 2
        )
        dcm[0, 1] = 2.0 * (q[1] * q[2] - q[0] * q[3])
        dcm[0, 2] = 2.0 * (q[1] * q[3] + q[0] * q[2])
        dcm[1, 0] = 2.0 * (q[1] * q[2] + q[0] * q[3])
        dcm[1, 1] = (
            q[0] ** 2 - 
            q[1] ** 2 + 
            q[2] ** 2 - 
            q[3] ** 2
        )
        dcm[1, 2] = 2.0*(q[2]*q[3] - q[0]*q[1])
        dcm[2, 0] = 2.0*(q[1]*q[3] - q[0]*q[2])
        dcm[2, 1] = 2.0*(q[2]*q[3] + q[0]*q[1])
        dcm[2, 2] = (
            q[0] ** 2 - 
            q[1] ** 2 - 
            q[2] ** 2 + 
            q[3] ** 2
        )

        return dcm
    
    def kardanwinkel2quaternion(self, psi, theta, phi):
        """Cardan angles to quaternion
        params: psi: yaw
        params: theta: pitch
        params: phi: roll
        """
        
        cr1 = numpy.cos(0.5 * psi)
        cr2 = numpy.cos(0.5 * theta)
        cr3 = numpy.cos(0.5 * phi)
        sr1 = numpy.sin(0.5 * psi)
        sr2 = numpy.sin(0.5 * theta)
        sr3 = numpy.sin(0.5 * phi)

        q0 = cr1 * cr2 * cr3 + sr1 * sr2 * sr3
        q1 = cr1 * cr2 * sr3 - sr1 * sr2 * cr3
        q2 = cr1 * sr2 * cr3 + sr1 * cr2 * sr3
        q3 = sr1 * cr2 * cr3 - cr1 * sr2 * sr3

        q = numpy.array([q0, q1, q2, q3])
        q = q / numpy.linalg.norm(q)
        return q

    def inverse(self, q):
        """Inverse quaternion 
        """
        qinv = numpy.array([q[0], -q[1], -q[2], -q[3]]) / numpy.linalg.norm(q)
        return qinv

    def quatMultiply(self, q, p):
        """Multiplication of quaternions 
        """
        Q = numpy.array([
            [ q[0], -q[1], -q[2], -q[3]],
            [ q[1],  q[0], -q[3],  q[2]],
            [ q[2],  q[3],  q[0], -q[1]],
            [ q[3], -q[2],  q[1],  q[0]]
        ])
        return Q@p

    def vectNormalize(self, q):
        """Normalize vector
        params: q: quaternion
        return: norm: norm of quaternion
        """
        norm = q / numpy.linalg.norm(q)
        return norm

    def rotationsmatrix2quaternion(self, R):    
        """Rotation matrix to quaternion
        params: R: rotationsmatrix 3x3
        return: q: quaternion
        """
        R11 = R[0, 0]
        R12 = R[0, 1]
        R13 = R[0, 2]
        R21 = R[1, 0]
        R22 = R[1, 1]
        R23 = R[1, 2]
        R31 = R[2, 0]
        R32 = R[2, 1]
        R33 = R[2, 2]
        # From page 68 of MotionGenesis book
        tr = R11 + R22 + R33

        if tr > R11 and tr > R22 and tr > R33:
            e0 = 0.5 * numpy.sqrt(1 + tr)
            r = 0.25 / e0
            e1 = (R32 - R23) * r
            e2 = (R13 - R31) * r
            e3 = (R21 - R12) * r
        elif R11 > R22 and R11 > R33:
            e1 = 0.5 * numpy.sqrt(1 - tr + 2 * R11)
            r = 0.25 / e1
            e0 = (R32 - R23) * r
            e2 = (R12 + R21) * r
            e3 = (R13 + R31) * r
        elif R22 > R33:
            e2 = 0.5 * numpy.sqrt(1 - tr + 2 * R22)
            r = 0.25 / e2
            e0 = (R13 - R31) * r
            e1 = (R12 + R21) * r
            e3 = (R23 + R32) * r
        else:
            e3 = 0.5 * numpy.sqrt(1 - tr + 2 * R33)
            r = 0.25 / e3
            e0 = (R21 - R12) * r
            e1 = (R13 + R31) * r
            e2 = (R23 + R32) * r

        # (e0, e1, e2, e3) = (qw, qx, qy, qz)
        q = numpy.array([e0, e1, e2, e3])
        q = q * numpy.sign(e0)
        q = q / numpy.sqrt(numpy.sum(
            q[0] ** 2 + 
            q[1] ** 2 + 
            q[2] ** 2 + 
            q[3] ** 2
        ))
        return q
    
if "__main__" == __name__:
    q = Quaternion()
