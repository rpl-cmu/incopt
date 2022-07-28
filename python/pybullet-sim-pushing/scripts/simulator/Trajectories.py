import pybullet as pb
import pybullet_data
import time
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

import time
import pdb


class Trajectories():

    def __init__(self, params):

        # read in sim params
        self.tstart = params['tstart']
        self.tend = params['tend']
        self.dt = params['dt']

        self.sim_length = np.int((self.tend - self.tstart) / self.dt)

        self.init_ee_pos = params['init_ee_pos']
        self.init_ee_ori = params['init_ee_ori']
        self.init_obj_pos = params['init_obj_pos']
        self.init_obj_ori = params['init_obj_ori']

    def get_traj_line(self, theta=0.0, randomize=False):

        traj_vec = np.zeros((self.sim_length, 3))
        pos_init = np.array([[self.init_ee_pos[0]], [self.init_ee_pos[1]],
                             [self.init_ee_pos[2]]])

        dir_vec = np.array([[np.sin(theta)], [np.cos(theta)], [0.0]])
        delta = 0.00005

        pos = pos_init
        for tstep in range(0, self.sim_length):
            dpos = delta * dir_vec
            pos += dpos
            traj_vec[tstep, :] += (pos[:, 0]).transpose()

        return traj_vec

    def get_traj_circle(self, theta=0.0, randomize=False):

        traj_vec = np.zeros((self.sim_length, 3))
        pos_init = np.array([[self.init_ee_pos[0]], [self.init_ee_pos[1]],
                             [self.init_ee_pos[2]]])

        dtheta = (np.pi / self.sim_length)
        dR = np.array([[np.cos(dtheta), np.sin(dtheta)],
                       [-np.sin(dtheta), np.cos(dtheta)]])

        pos = pos_init
        for tstep in range(0, self.sim_length):
            pos[0:2] = np.matmul(dR, pos[0:2])
            traj_vec[tstep, :] = pos.transpose()

        return traj_vec
