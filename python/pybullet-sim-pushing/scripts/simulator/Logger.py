import pybullet as pb
import pybullet_data

import time
from datetime import datetime
import numpy as np
import json

import os
import pdb


class Logger():

    def __init__(self, params):

        self.params = params

        # read in sim params
        self.tstart = params['tstart']
        self.tend = params['tend']
        self.dt = params['dt']
        self.sim_length = np.int((self.tend - self.tstart) / self.dt)

        # obj, endeff geometric params
        self.ee_radius = params['ee_radius']
        self.block_size_x = params['block_size_x']
        self.block_size_y = params['block_size_y']

        # time steps
        self.t = np.zeros((self.sim_length, 1))

        # endeffector poses
        self.ee_pos = np.zeros((self.sim_length, 3))
        self.ee_ori = np.zeros((self.sim_length, 4))  # quaternion
        self.ee_ori_mat = np.zeros((self.sim_length, 9))  # rotation matrix
        self.ee_ori_rpy = np.zeros((self.sim_length, 3))  # r,p,y angles

        # object poses
        self.obj_pos = np.zeros((self.sim_length, 3))
        self.obj_ori = np.zeros((self.sim_length, 4))  # quaternion
        self.obj_ori_mat = np.zeros((self.sim_length, 9))  # rotation matrix
        self.obj_ori_rpy = np.zeros((self.sim_length, 3))  # r,p,y angles

        # contact pos/force measurements (A: endeff, B: object)
        self.contact_flag = np.zeros((self.sim_length, 1))
        self.contact_pos_onA = np.zeros((self.sim_length, 3))
        self.contact_pos_onB = np.zeros((self.sim_length, 3))
        self.contact_normal_onB = np.zeros((self.sim_length, 3))
        self.contact_distance = np.zeros((self.sim_length, 1))
        self.contact_normal_force = np.zeros((self.sim_length, 1))

        # contact friction measurements
        self.lateral_friction1 = np.zeros((self.sim_length, 1))
        self.lateral_frictiondir1 = np.zeros((self.sim_length, 3))
        self.lateral_friction2 = np.zeros((self.sim_length, 1))
        self.lateral_frictiondir2 = np.zeros((self.sim_length, 3))

    def get_shape_poly_vertices(self, shape_id):

        if (shape_id == "rect"):
            poly_vertices = np.array([[-0.5*self.block_size_x, -0.5*self.block_size_y],
                                      [-0.5*self.block_size_x, 0.5*self.block_size_y],
                                      [0.5*self.block_size_x, 0.5*self.block_size_y],
                                      [0.5*self.block_size_x, -0.5*self.block_size_y]])
            # poly_vertices = poly_vertices.transpose()  # 2 x nv

        return poly_vertices

    def get_contact_force(self):

        # contact forces (applied by endeff on object)
        f_normal = self.contact_normal_force * -self.contact_normal_onB[:, 0:2]
        f_lateral = self.lateral_friction1 * self.lateral_frictiondir1[:, 0:2]
        contact_forces_2d = f_normal + f_lateral

        return contact_forces_2d

    def update_logged_params(self):
        self.params['tau_max'] = self.params['mu_g'] * np.sqrt(self.block_size_x**2 + self.block_size_y**2) * \
            self.params['block_mass'] * self.params['gravity']
        self.params['f_max'] = self.params['mu_g'] * \
            self.params['block_mass'] * self.params['gravity']

    def save_data2d_json(self, dstfile):

        self.update_logged_params()

        ee_poses_2d = np.zeros((self.sim_length, 3))
        obj_poses_2d = np.zeros((self.sim_length, 3))
        ee_poses_2d[:, 0:2] = self.ee_pos[:, 0:2]
        ee_poses_2d[:, 2] = self.ee_ori_rpy[:, 2]
        obj_poses_2d[:, 0:2] = self.obj_pos[:, 0:2]
        obj_poses_2d[:, 2] = self.obj_ori_rpy[:, 2]

        obj_poly_shape = self.get_shape_poly_vertices('rect')
        contact_forces_2d = self.get_contact_force()

        #  use .tolist() to serialize np arrays
        data = {'params': self.params,
                'obj_poly_shape': obj_poly_shape.tolist(),
                'ee_poses_2d': ee_poses_2d.tolist(),
                'obj_poses_2d': obj_poses_2d.tolist(),
                'contact_flag': self.contact_flag.tolist(),
                'contact_normal_dirs2d': (-self.contact_normal_onB[:, 0:2]).tolist(),
                'contact_normal_forces': self.contact_normal_force.tolist(),
                'contact_lateral_dirs2d': (self.lateral_frictiondir1[:, 0:2]).tolist(),
                'contact_lateral_forces': self.lateral_friction1.tolist(),
                'contact_forces_2d': contact_forces_2d.tolist(),
                'contact_points_gt_2d': (self.contact_pos_onB[:, 0:2]).tolist()}

        with open(dstfile, 'w') as outfile:
            json.dump(data, outfile, indent=4)

        print("Finished writing json dataset: {0}".format(dstfile))
