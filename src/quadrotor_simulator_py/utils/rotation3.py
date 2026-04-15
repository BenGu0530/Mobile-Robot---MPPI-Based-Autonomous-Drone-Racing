import numpy as np

from numpy import arctan2 as atan2
from numpy import arcsin as asin
from numpy import cos as cos
from numpy import sin as sin

from quadrotor_simulator_py.utils.quaternion import Quaternion


class Rotation3:

    def __init__(self, R=None):
        self.R = None

        if R is None:
            self.R = np.eye(3)
        else:
            self.R = R

    def to_euler_zyx(self):
        """ Convert self.R to Z-Y-X euler angles

        Output:
            zyx: 1x3 numpy array containing euler angles.
                The order of angles should be phi, theta, psi, where
                roll == phi, pitch == theta, yaw == psi
        """

        phi = atan2(self.R[2, 1], self.R[2, 2])
        theta = asin(-self.R[2, 0])
        psi = atan2(self.R[1, 0], self.R[0, 0])

        # TODO: Assignment 1, Problem 1.1
        return np.array([phi, theta, psi])

    @classmethod
    def from_euler_zyx(self, zyx):
        """ Convert euler angle rotation representation to 3x3
                rotation matrix. The input is represented as 
                np.array([roll, pitch, yaw]).
        Arg:
            zyx: 1x3 numpy array containing euler angles

        Output:
            Rot: 3x3 rotation matrix (numpy)
        """

        # TODO: Assignment 1, Problem 1.2

        phi = zyx[0]
        theta = zyx[1]
        psi = zyx[2]

        r0 = np.array([cos(psi)*cos(theta),
                       cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi),
                       cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi)])
        r1 = np.array([sin(psi)*cos(theta),
                       sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi),
                       sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi)])
        r2 = np.array([-sin(theta),
                       cos(theta)*sin(phi),
                       cos(theta)*cos(phi)])

        Rot = Rotation3(np.array([r0, r1, r2]))
        #Rot.R = np.eye(3)
        return Rot

    def roll(self):
        """ Extracts the phi component from the rotation matrix

        Output:
            phi: scalar value representing phi
        """

        # TODO: Assignment 1, Problem 1.3

        return atan2(self.R[2, 1], self.R[2, 2])

    def pitch(self):
        """ Extracts the theta component from the rotation matrix

        Output:
            theta: scalar value representing theta
        """

        # TODO: Assignment 1, Problem 1.4

        return asin(-self.R[2, 0])

    def yaw(self):
        """ Extracts the psi component from the rotation matrix

        Output:
            theta: scalar value representing psi
        """

        # TODO: Assignment 1, Problem 1.5

        return atan2(self.R[1, 0], self.R[0, 0])

    @classmethod
    def from_quat(self, q):
        """ Calculates the 3x3 rotation matrix from a quaternion
                parameterized as (w,x,y,z).

        Output:
            Rot: 3x3 rotation matrix represented as numpy matrix
        """

        # TODO: Assignment 1, Problem 1.6

        w = q.w()
        x = q.x()
        y = q.y()
        z = q.z()
        r0 = np.array([1 - 2*(y**2 + z**2),
                       2*(x*y - z*w),
                       2*(x*z + y*w)])
        r1 = np.array([2*(x*y + z*w),
                       1 - 2*(x**2 + z**2),
                       2*(y*z - x*w)])
        r2 = np.array([2*(x*z - y*w),
                       2*(y*z + x*w),
                       1 - 2*(x**2 + y**2)])

        Rot = Rotation3(np.array([r0, r1, r2]))
        #Rot.R = np.eye(3)
        return Rot

    def to_quat(self):
        """ Calculates a quaternion from the class variable
                self.R and returns it

        Output:
            q: An instance of the Quaternion class parameterized
                as [w, x, y, z]
        """

        # TODO: Assignment 1, Problem 1.7

        euler = self.to_euler_zyx()
        phi_by_2 = euler[0]/2
        theta_by_2 = euler[1]/2
        psi_by_2 = euler[2]/2
        w = cos(phi_by_2)*cos(theta_by_2)*cos(psi_by_2) + sin(phi_by_2)*sin(theta_by_2)*sin(psi_by_2)
        x = sin(phi_by_2)*cos(theta_by_2)*cos(psi_by_2) - cos(phi_by_2)*sin(theta_by_2)*sin(psi_by_2)
        y = cos(phi_by_2)*sin(theta_by_2)*cos(psi_by_2) + sin(phi_by_2)*cos(theta_by_2)*sin(psi_by_2)
        z = cos(phi_by_2)*cos(theta_by_2)*sin(psi_by_2) - sin(phi_by_2)*sin(theta_by_2)*cos(psi_by_2)
        return Quaternion([w, x, y, z])
