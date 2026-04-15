#!/usr/bin/env python
import numpy as np
from numpy import sin, cos
from numpy.linalg import norm
import yaml

from quadrotor_simulator_py.quadrotor_control.state import State
from quadrotor_simulator_py.quadrotor_control.trackingerror import TrackingError
from quadrotor_simulator_py.quadrotor_model.mixer import QuadMixer
from quadrotor_simulator_py.quadrotor_control.cascaded_command import CascadedCommand
from quadrotor_simulator_py.utils import Quaternion
from quadrotor_simulator_py.utils import shortest_angular_distance


class QuadrotorPositionControllerPD:

    def __init__(self, yaml_file):
        self.zw = np.array([[0], [0], [1]])  # unit vector [0, 0, 1]^{\top}
        self.gravity_norm = 9.81
        self.current_state = State()
        self.state_ref = State()
        self.tracking_error = TrackingError()

        self.Rdes = np.eye(3)
        self.Rcurr = None
        self.accel_des = 0.0
        self.angvel_des = np.zeros((3, 1))
        self.angacc_des = np.zeros((3, 1))
        self.mass = 0.0

        data = []
        with open(yaml_file, 'r') as stream:
            try:
                data = yaml.safe_load(stream)
            except yaml.YamlError as exc:
                print(exc)

        self.mass = data['mass']

        self._Kx = np.eye(3)
        self._Kx[0, 0] = data['gains']['pos']['x']
        self._Kx[1, 1] = data['gains']['pos']['y']
        self._Kx[2, 2] = data['gains']['pos']['z']

        self._Kv = np.eye(3)
        self._Kv[0, 0] = data['gains']['vel']['x']
        self._Kv[1, 1] = data['gains']['vel']['y']
        self._Kv[2, 2] = data['gains']['vel']['z']

        self.gravity_norm = data['gravity_norm']

    def update_state(self):
        self.Rcurr = self.current_state.rot

    def set_current_state(self, state_in):
        self.current_state = state_in
        self.update_state()

    def get_state(self):
        return self.current_state

    def set_reference_state(self, ref_in):
        self.state_ref = ref_in

    def compute_body_z_accel(self, a_des, R_curr):
        """ Calculates the body-frame z-acceleration

        Args:
            a_des: 3x1 numpy array representing the desired acceleration
            R_curr: 3x3 rotation matrix representing Rwb

        Output:
            u: scalar value representing body-frame z-acceleration
        """

        # TODO: Assignment 1, Problem 2.1
        zb = R_curr@np.array([0,0,1])
        return np.dot(zb, a_des)

    def compute_orientation(self, a_des, yaw_ref):
        """ Calculates the desired orientation

        Args:
            a_des: 3x1 numpy array representing the desired acceleration
            yaw_ref: yaw reference

        Output:
            R_des: 3x3 numpy matrix representing desired orientation
        """

        # TODO: Assignment 1, Problem 2.2
        yc = np.array([-np.sin(yaw_ref),np.cos(yaw_ref),0])
        zbdes = a_des/np.linalg.norm(a_des)
        t1 = np.cross(yc.flatten(), zbdes.flatten())
        xbdes = t1/np.linalg.norm(t1)
        ybdes = np.cross(zbdes.flatten(), xbdes.flatten())

        R_des = np.stack([xbdes.flatten(), ybdes.flatten(), zbdes.flatten()], axis=1)
        return R_des

    def compute_hod_refs(self, acc_vec_des, flat_ref, R_des):
        """ Calculates the desired angular velocities and accelerations.

        Args:
            acc_vec_des: 3x1 numpy array representing the desired acceleration
            flat_ref: class instance of State() containing the trajectory reference
            R_des: desired rotation

        Output:
            angvel_des: 3x1 numpy array representing desired angular velocity
            angacc_des: 3x1 numpy array representing desired angular acceleration
        """

        # TODO: Assignment 1, Problem 2.3

        jref = flat_ref.jerk.flatten()
        xbdes = R_des[:,0]
        ybdes = R_des[:,1]
        zbdes = R_des[:,2]
        # zb = self.current_state.rot@np.array([0,0,1])
        # c = np.dot(zb, acc_vec_des)
        # c = np.linalg.norm(acc_vec_des)
        c = np.dot(zbdes, acc_vec_des)
        wx = -np.dot(ybdes, jref)/c
        wy = np.dot(xbdes,jref)/c
        psi = flat_ref.yaw
        psidot = flat_ref.dyaw
        cpsi = np.cos(psi)
        spsi = np.sin(psi)
        xc = np.array([cpsi, spsi, 0])
        yc = np.array([-spsi, cpsi, 0])
        wz = (psidot*np.dot(xc,xbdes) + wy*np.dot(yc,zbdes))/np.linalg.norm(np.cross(yc.flatten(),zbdes.flatten()))

        cdot = np.dot(zbdes, jref)
        s = flat_ref.snap.flatten()
        ax = (-np.dot(ybdes,s) - 2*cdot*wx + c*wy*wz)/c
        ay = (np.dot(xbdes,s)-2*cdot*wy - c*wx*wz)/c
        psidot2 = flat_ref.d2yaw
        az1 = (psidot2*np.dot(xc,xbdes)-2*psidot*wy*np.dot(xc,zbdes))
        az2 = (2*psidot*wz*np.dot(xc,ybdes) - wx*wy*np.dot(yc,ybdes))
        az3 = (-wx*wz*np.dot(yc,zbdes) + ay*np.dot(yc,zbdes))
        az = (az1+az2+az3)/np.linalg.norm(np.cross(yc.flatten(),zbdes.flatten()))


        angvel_des = np.array([wx,wy,wz])
        angacc_des = np.array([ax,ay,az])
        return (angvel_des, angacc_des)

    def compute_command(self):
        """ This function contains the following functionality:
                1. Computes the PD feedback-control terms from the position
                   and velocity control errors.
                2. Computes the desired rotation using compute_orientation.
                3. Applies the thrust command to the body frame using
                   compute_body_z_accel
                4. Calculates the desired angular velocities and accelerations.
        """

        # TODO: Assignment 1, Problem 2.4
        ades = -self._Kx@(self.current_state.pos - self.state_ref.pos) -self._Kv@(self.current_state.vel-self.state_ref.vel) + self.state_ref.acc + self.gravity_norm*self.zw
        #print( -self._Kx@(self.current_state.pos - self.state_ref.pos))
        # Tdes = self.compute_body_z_accel(self.accel_des, self.Rcurr)*self.mass
        self.accel_des = self.compute_body_z_accel(ades, self.Rcurr)
        self.Rdes = self.compute_orientation(ades, self.state_ref.yaw)
        self.angvel_des, self.angacc_des = self.compute_hod_refs(ades, self.state_ref, self.Rdes)
        # print(self.angvel_des, self.angacc_des)


    def get_cascaded_command(self):
        casc_cmd = CascadedCommand()
        casc_cmd.thrust_des = self.mass * self.accel_des
        casc_cmd.Rdes = self.Rdes
        casc_cmd.angvel_des = self.angvel_des
        casc_cmd.angacc_des = self.angacc_des
        return casc_cmd

    def get_tracking_error(self):
        return self.tracking_error

    def update_tracking_error(self):
        self.tracking_error = TrackingError()
        self.tracking_error.pos_des = self.state_ref.pos
        self.tracking_error.vel_des = self.state_ref.vel
        self.tracking_error.acc_des = self.state_ref.acc
        self.tracking_error.jerk_des = self.state_ref.jerk
        self.tracking_error.snap_des = self.state_ref.snap
        self.tracking_error.yaw_des = self.state_ref.yaw
        self.tracking_error.dyaw_des = self.state_ref.dyaw
        self.tracking_error.pos_err = self.current_state.pos - self.state_ref.pos
        self.tracking_error.vel_err = self.current_state.vel - self.state_ref.vel
        self.tracking_error.yaw_err = shortest_angular_distance(
            self.current_state.yaw, self.state_ref.yaw)
        self.tracking_error.dyaw_err = self.current_state.dyaw - self.state_ref.dyaw

    def run_ctrl(self):

        # get updated state
        self.update_state()

        # calculate the command
        self.compute_command()

        # update tracking error
        self.update_tracking_error()
