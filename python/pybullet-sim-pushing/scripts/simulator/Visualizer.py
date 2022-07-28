import utils as utils

import pybullet as pb
import pybullet_data

import time
from datetime import datetime
import numpy as np
import json

from shapely.geometry import Point, Polygon, LinearRing
from shapely.ops import nearest_points

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.gridspec import GridSpec

import os
import pdb

plt.rcParams.update({'font.size': 18})


class Visualizer():

    def __init__(self, params, logger):

        self.params = params
        self.logger = logger

        # read in sim params
        self.tstart = params['tstart']
        self.tend = params['tend']
        self.dt = params['dt']
        self.sim_length = np.int((self.tend - self.tstart) / self.dt)
        self.tspan = np.arange(self.tstart, self.tend, self.dt)

        # obj, endeff geometric params
        self.ee_radius = params['ee_radius']
        self.block_size_x = params['block_size_x']
        self.block_size_y = params['block_size_y']

        # time steps
        self.t = np.zeros((self.sim_length, 1))

        # endeffector poses
        self.ee_pos = self.logger.ee_pos
        self.ee_ori = self.logger.ee_ori
        self.ee_ori_mat = self.logger.ee_ori_mat
        self.ee_ori_rpy = self.logger.ee_ori_rpy

        # object poses
        self.obj_pos = self.logger.obj_pos
        self.obj_ori = self.logger.obj_ori
        self.obj_ori_mat = self.logger.obj_ori_mat
        self.obj_ori_rpy = self.logger.obj_ori_rpy

        # contact pos/force measurements (A: endeff, B: object)
        self.contact_flag = self.logger.contact_flag
        self.contact_pos_onA = self.logger.contact_pos_onA
        self.contact_pos_onB = self.logger.contact_pos_onB
        self.contact_normal_onB = self.logger.contact_normal_onB
        self.contact_distance = self.logger.contact_distance
        self.contact_normal_force = self.logger.contact_normal_force

        # contact friction measurements
        self.lateral_friction1 = self.logger.lateral_friction1
        self.lateral_frictiondir1 = self.logger.lateral_frictiondir1
        self.lateral_friction2 = self.logger.lateral_friction2
        self.lateral_frictiondir2 = self.logger.lateral_frictiondir2

    def visualize_contact_data(self, save_fig=False):
        """Visualize contact info data returned from getContactPoints()"""

        incontact_idxs = np.argwhere(self.contact_flag == 1)
        incontact_idxs = incontact_idxs[:, 0]

        dst_dir = "../local/outputs/contact_info/"
        if (save_fig):
            cmd = "mkdir -p {0}".format(dst_dir)
            os.popen(cmd, 'r')

        fig = plt.figure(figsize=(10, 8))
        step_skip = 50
        for i in range(0, len(incontact_idxs), step_skip):

            idx = incontact_idxs[i]

            plt.xlim((-0.6, 0.6))
            plt.ylim((-0.2, 0.8))
            # plt.xlim((-1, 2))
            # plt.ylim((-0.1, 2))
            sz_arw = 0.05

            # plot end effector (A: endeff, B: object)
            plt.plot(self.ee_pos[idx, 0], self.ee_pos[idx, 1],
                     color='green', marker='o')
            plt.plot(self.contact_pos_onA[idx, 0], self.contact_pos_onA[idx, 1],
                     color='green', marker='o')
            circ = Circle((self.ee_pos[idx, 0], self.ee_pos[idx, 1]), self.ee_radius,
                          facecolor='None', edgecolor='grey', linestyle='-', linewidth=2)
            plt.gca().add_patch(circ)

            # plot object pose
            yaw = self.obj_ori_rpy[idx, 2]
            R = np.array([[np.cos(yaw), -np.sin(yaw)],
                          [np.sin(yaw), np.cos(yaw)]])
            offset = np.matmul(R, np.array(
                [[0.5*self.block_size_x], [0.5*self.block_size_y]]))
            xb = self.obj_pos[idx, 0] - offset[0]
            yb = self.obj_pos[idx, 1] - offset[1]
            rect = Rectangle((xb, yb), self.block_size_x, self.block_size_y, angle=(
                np.rad2deg(yaw)), facecolor='None', edgecolor='grey', linewidth=2)
            plt.gca().add_patch(rect)

            plt.plot(self.contact_pos_onB[idx, 0], self.contact_pos_onB[idx, 1],
                     color='red', marker='o')
            plt.arrow(self.contact_pos_onB[idx, 0], self.contact_pos_onB[idx, 1],
                      -sz_arw * self.contact_normal_onB[idx, 0],
                      -sz_arw * self.contact_normal_onB[idx, 1],
                      head_width=5e-3, color='black')

            # # lateral friction forces
            plt.arrow(self.contact_pos_onA[idx, 0], self.contact_pos_onA[idx, 1],
                      sz_arw * self.lateral_frictiondir1[idx, 0],
                      sz_arw * self.lateral_frictiondir1[idx, 1],
                      head_width=5e-3, color='green')
            # plt.arrow(self.contact_pos_onA[idx, 0], self.contact_pos_onB[idx, 1],
            #   sz_arw * self.lateral_frictiondir2[idx, 0],
            #   sz_arw * self.lateral_frictiondir2[idx, 1],
            #   head_width=5e-3, color='red')

            plt.draw()
            plt.pause(1e-6)

            if(save_fig):
                plt.savefig('{0}/{1:06d}.png'.format(dst_dir, i))

            plt.cla()

    def plot_contact_force_data(self, save_fig=False):
        """ Visualize force data returned from getContactPoints() """

        fig = plt.figure(constrained_layout=True, figsize=(10, 8))
        nrows = 3
        ncols = 1
        gs = GridSpec(nrows, ncols, figure=fig)

        incontact_idxs = np.argwhere(self.contact_flag == 1)
        incontact_idxs = incontact_idxs[:, 0]

        # subsample
        step = 1
        skip = 50  # skip initial impulse iterations
        incontact_idxs = incontact_idxs[skip:-1:step]

        ax = [None] * (nrows*ncols)
        ax[0] = fig.add_subplot(gs[0, 0])
        ax[1] = fig.add_subplot(gs[1, 0])
        ax[2] = fig.add_subplot(gs[2, 0])

        # contact forces (applied by endeff on object)
        f_contact_normal = (self.contact_normal_force[incontact_idxs, :]
                            ) * -self.contact_normal_onB[incontact_idxs, 0:2]
        f_contact_lateral = (self.lateral_friction1[incontact_idxs, :]
                             ) * self.lateral_frictiondir1[incontact_idxs, 0:2]
        f_contact = f_contact_normal + f_contact_lateral

        ax[0].plot(self.tspan[incontact_idxs],
                   f_contact_normal[:, 0], color='red', label='f_x')
        ax[0].plot(self.tspan[incontact_idxs],
                   f_contact_normal[:, 1], color='green', label='f_y')

        ax[1].plot(self.tspan[incontact_idxs],
                   f_contact_lateral[:, 0], color='red', label='f_x')
        ax[1].plot(self.tspan[incontact_idxs],
                   f_contact_lateral[:, 1], color='green', label='f_y')

        ax[2].plot(self.tspan[incontact_idxs],
                   f_contact[:, 0], color='red', label='f_x')
        ax[2].plot(self.tspan[incontact_idxs],
                   f_contact[:, 1], color='green', label='f_y')

        ax[0].set_title('force_contact_normal')
        ax[1].set_title('force_contact_lateral')
        ax[2].set_title('force_contact')

        ax[0].legend(loc='upper left')
        ax[1].legend(loc='upper left')
        ax[2].legend(loc='upper left')

        plt.show()

    def proj_ee_centers(self):
        """ projects endeff center on to object polygon """

        poly_obj = Polygon([(-0.0-0.5*self.block_size_x, -0.5*self.block_size_y),
                            (-0.0-0.5*self.block_size_x, 0.5*self.block_size_y),
                            (-0.0+0.5*self.block_size_x, 0.5*self.block_size_y),
                            (-0.0+0.5*self.block_size_x, -0.5*self.block_size_y)])

        ee_centers_poly = np.zeros((self.sim_length, 2))

        for tstep in range(0, self.sim_length, 1):

            if (self.contact_flag[tstep] == False):
                continue

            frame_pose2d = np.array([[self.obj_pos[tstep, 0]], [self.obj_pos[tstep, 1]],
                                     [self.obj_ori_rpy[tstep, 2]]])
            ee_center__world = np.array([[self.ee_pos[tstep, 0]],
                                         [self.ee_pos[tstep, 1]]])

            ee_center__obj = utils.transform_to_frame2d(
                ee_center__world, frame_pose2d)

            # proj method 1 (pt2poly): using nearest_points
            dist = Point(ee_center__obj[0],
                         ee_center__obj[1]).distance(poly_obj)
            ee_center_poly__obj, p2 = nearest_points(poly_obj, Point(
                ee_center__obj[0], ee_center__obj[1]))

            ee_center_poly__world = utils.transform_from_frame2d(
                np.array([[ee_center_poly__obj.x], [ee_center_poly__obj.y]]), frame_pose2d)
            ee_centers_poly[tstep, :] = ee_center_poly__world.transpose()

        return ee_centers_poly

    def visualize_contact_factor__world(self, save_fig=False):
        """ Contact: Verify contact factor points visually using groundtruth values (world frame). Ref: Yu '18 """

        incontact_idxs = np.argwhere(self.contact_flag == 1)
        incontact_idxs = incontact_idxs[:, 0]

        # project endeff centers on object polygon
        ee_centers_poly = self.proj_ee_center()

        dst_dir = "../local/outputs/contact_factor__world/traj_circle/"
        if (save_fig):
            cmd = "mkdir -p {0}".format(dst_dir)
            os.popen(cmd, 'r')

        fig = plt.figure(figsize=(10, 8))
        skip = 50
        for tstep in range(0, self.sim_length, skip):

            if (self.contact_flag[tstep] == False):
                continue

            plt.xlim((-1.0, 0.6))
            plt.ylim((-0.2, 0.8))
            sz_arw = 0.05

            # plot end effector (A: endeff, B: object)
            plt.plot(self.ee_pos[tstep, 0], self.ee_pos[tstep, 1],
                     color='green', marker='o')
            plt.plot(ee_centers_poly[tstep, 0], ee_centers_poly[tstep, 1],
                     color='blue', marker='o')
            circ = Circle((self.ee_pos[tstep, 0], self.ee_pos[tstep, 1]), self.ee_radius,
                          facecolor='None', edgecolor='grey', linestyle='-')
            plt.gca().add_patch(circ)

            # plot contact point (computed using force dxn)
            pt_contact__world = np.array([[self.ee_pos[tstep, 0] - self.ee_radius*self.contact_normal_onB[tstep, 0]],
                                          [self.ee_pos[tstep, 1] - self.ee_radius*self.contact_normal_onB[tstep, 1]]])
            plt.plot(
                pt_contact__world[0], pt_contact__world[1], color='red', marker='o')
            plt.arrow(self.ee_pos[tstep, 0], self.ee_pos[tstep, 1],
                      -(self.ee_radius) * self.contact_normal_onB[tstep, 0],
                      -(self.ee_radius) * self.contact_normal_onB[tstep, 1],
                      head_width=5e-3, color='black')

            # test: contact point returned from getContactPoints()
            # pt_contact_test__world = np.array(
            #     [[self.contact_pos_onB[tstep, 0]], [self.contact_pos_onB[tstep, 1]]])
            # plt.plot(
            #     pt_contact_test__world[0], pt_contact_test__world[1], color='yellow', marker='o', alpha=1.0)

            # plot object pose
            yaw = self.obj_ori_rpy[tstep, 2]
            R = np.array([[np.cos(yaw), -np.sin(yaw)],
                          [np.sin(yaw), np.cos(yaw)]])
            offset = np.matmul(R, np.array(
                [[0.5*self.block_size_x], [0.5*self.block_size_y]]))
            xb = self.obj_pos[tstep, 0] - offset[0]
            yb = self.obj_pos[tstep, 1] - offset[1]
            rect = Rectangle((xb, yb), self.block_size_x, self.block_size_y, angle=(
                np.rad2deg(yaw)), facecolor='None', edgecolor='grey')
            plt.gca().add_patch(rect)

            plt.draw()
            plt.pause(1e-12)

            if(save_fig):
                plt.savefig("{0}/{1:06d}.png".format(dst_dir, tstep))

            plt.cla()

    def qs_model1(self, tstep, prev_step):
        """ QS model: quasi-static model using forces. Ref: Lynch '92 """

        tau_max = self.params['mu_g'] * np.sqrt(self.block_size_x**2 + self.block_size_y**2) * \
            self.params['block_mass'] * self.params['gravity']
        f_max = self.params['mu_g'] * \
            self.params['block_mass'] * self.params['gravity']
        c_sq = (tau_max / f_max) ** 2

        # obj, endeff poses at curr, prev time steps
        ee_pose2d_prev__world = np.array([self.ee_pos[prev_step, :][0],
                                          self.ee_pos[prev_step, :][1], self.ee_ori_rpy[prev_step, :][2]])
        obj_pose2d_prev__world = np.array([self.obj_pos[prev_step, :][0],
                                           self.obj_pos[prev_step, :][1], self.obj_ori_rpy[prev_step, :][2]])
        obj_pose2d_curr__world = np.array(
            [self.obj_pos[tstep, :][0], self.obj_pos[tstep, :][1], self.obj_ori_rpy[tstep, :][2]])
        ee_pose2d_curr__world = np.array(
            [self.ee_pos[tstep, :][0], self.ee_pos[tstep, :][1], self.ee_ori_rpy[tstep, :][2]])

        # measurements at current time step, contact forces as applied by endeff on object
        contact_point__world = self.contact_pos_onB[tstep, 0:2]
        contact_normal__world = -self.contact_normal_onB[tstep, 0:2]
        f_normal = self.contact_normal_force[tstep] * - \
            self.contact_normal_onB[tstep, 0:2]
        f_lateral = self.lateral_friction1[tstep] * \
            self.lateral_frictiondir1[tstep, 0:2]
        contact_force__world = f_normal + f_lateral

        # transform measurement to object frame
        frame_pose2d = np.array([[obj_pose2d_curr__world[0]], [obj_pose2d_curr__world[1]],
                                 [obj_pose2d_curr__world[2]]])
        contact_point__obj = utils.transform_to_frame2d(
            contact_point__world[:, None], obj_pose2d_curr__world[:, None])
        contact_normal__obj = utils.rotate_to_frame2d(
            contact_normal__world[:, None], obj_pose2d_curr__world[:, None])
        contact_force__obj = utils.rotate_to_frame2d(
            contact_force__world[:, None], obj_pose2d_curr__world[:, None])

        # compute moment tau_z
        tau_z = utils.cross2d(contact_point__obj,
                              contact_force__obj)  # r x f

        # compute velocities
        vel__obj = utils.rotate_to_frame2d(
            obj_pose2d_curr__world[0:2] - obj_pose2d_prev__world[0:2], frame_pose2d)
        omega = (obj_pose2d_curr__world[2] - obj_pose2d_prev__world[2])
        omega = (omega + np.pi) % (2 * np.pi) - np.pi  # wrap2pi

        f_x = contact_force__obj[0]
        f_y = contact_force__obj[1]
        v_x = vel__obj[0]
        v_y = vel__obj[1]

        # note: multiplicative form masks errors when omega/tau_z values are low
        error = [v_x * tau_z - c_sq * f_x * omega,
                 v_y * tau_z - c_sq * f_y * omega]
        # error = [v_x / omega - c_sq * f_x / tau_z,
        #  v_y / omega - c_sq * f_y / tau_z]
        twist = [v_x, v_y, omega]
        force = [f_x, f_y]

        return (error, twist, force)

    def qs_model2(self, tstep, prev_step):
        """ QS model: velocity-only quasi-static model. Ref: Zhou '17 """

        tau_max = self.params['mu_g'] * np.sqrt(self.block_size_x**2 + self.block_size_y**2) * \
            self.params['block_mass'] * self.params['gravity']
        f_max = self.params['mu_g'] * \
            self.params['block_mass'] * self.params['gravity']

        # obj, endeff poses at prev, curr time steps
        ee_pose2d_prev__world = np.array([self.ee_pos[prev_step, :][0],
                                          self.ee_pos[prev_step, :][1], self.ee_ori_rpy[prev_step, :][2]])
        obj_pose2d_prev__world = np.array([self.obj_pos[prev_step, :][0],
                                           self.obj_pos[prev_step, :][1], self.obj_ori_rpy[prev_step, :][2]])
        obj_pose2d_curr__world = np.array(
            [self.obj_pos[tstep, :][0], self.obj_pos[tstep, :][1], self.obj_ori_rpy[tstep, :][2]])
        ee_pose2d_curr__world = np.array(
            [self.ee_pos[tstep, :][0], self.ee_pos[tstep, :][1], self.ee_ori_rpy[tstep, :][2]])
        contact_point_prev__world = self.contact_pos_onB[prev_step, 0:2]

        # measurements at current time step, contact forces as applied by endeff on object
        contact_point__world = self.contact_pos_onB[tstep, 0:2]
        contact_normal__world = -self.contact_normal_onB[tstep, 0:2]
        f_normal = self.contact_normal_force[tstep] * - \
            self.contact_normal_onB[tstep, 0:2]
        f_lateral = self.lateral_friction1[tstep] * \
            self.lateral_frictiondir1[tstep, 0:2]
        contact_force__world = f_normal + f_lateral

        # transform measurement to object frame
        frame_pose2d = np.array([[obj_pose2d_curr__world[0]], [obj_pose2d_curr__world[1]],
                                 [obj_pose2d_curr__world[2]]])
        contact_point__obj = utils.transform_to_frame2d(
            contact_point__world[:, None], obj_pose2d_curr__world[:, None])
        contact_normal__obj = utils.rotate_to_frame2d(
            contact_normal__world[:, None], obj_pose2d_curr__world[:, None])
        contact_force__obj = utils.rotate_to_frame2d(
            contact_force__world[:, None], obj_pose2d_curr__world[:, None])

        # compute velocities
        vel_body__obj = utils.rotate_to_frame2d(
            obj_pose2d_curr__world[0:2] - obj_pose2d_prev__world[0:2], frame_pose2d)
        vel_contact__obj = utils.rotate_to_frame2d(
            contact_point__world[0:2] - contact_point_prev__world[0:2], frame_pose2d)

        omega = (obj_pose2d_curr__world[2] - obj_pose2d_prev__world[2])
        omega = (omega + np.pi) % (2 * np.pi) - np.pi  # wrap2pi
        v_x = vel_body__obj[0]
        v_y = vel_body__obj[1]

        v_px = vel_contact__obj[0]
        v_py = vel_contact__obj[1]

        px = contact_point__obj[0, 0]
        py = contact_point__obj[1, 0]

        V = np.array([[v_x], [v_y], [omega]])
        Vp = np.array([[v_px], [v_py], [0]])
        D = np.array([[1, 0, -py], [0, 1, px],
                      [-f_max**2 * py, f_max**2 * px, -tau_max**2]])

        error = (np.matmul(D, V) - Vp).transpose()
        twist_object = [v_x, v_y, omega]
        twist_contact = [v_px, v_py, 0]

        return (error, twist_object, twist_contact)

    def plot_qs_push_dynamics_errors(self):

        step_skip = 1
        init_skip = 100  # initial impulse iterations
        step_range = range(init_skip, self.sim_length, step_skip)
        nsteps = len(step_range)

        error_vec_1 = np.zeros((nsteps, 2))
        twist_object_vec_1 = np.zeros((nsteps, 3))
        force_vec_1 = np.zeros((nsteps, 2))

        error_vec_2 = np.zeros((nsteps, 3))
        twist_object_vec_2 = np.zeros((nsteps, 3))
        twist_contact_vec_2 = np.zeros((nsteps, 3))

        idx = 0
        for tstep in step_range:

            if (self.contact_flag[tstep] == False):
                continue

            prev_step = np.maximum(0, tstep-step_skip)

            [error_1, twist_object_1, force_1] = self.qs_model1(
                tstep, prev_step)
            [error_2, twist_object_2, twist_contact_2] = self.qs_model2(
                tstep, prev_step)

            error_vec_1[idx, :] = error_1
            twist_object_vec_1[idx, :] = twist_object_1
            force_vec_1[idx, :] = force_1

            error_vec_2[idx, :] = error_2
            twist_object_vec_2[idx, :] = twist_object_2
            twist_contact_vec_2[idx, :] = twist_contact_2

            idx = idx + 1

        fig = plt.figure(constrained_layout=True)
        nrows = 3
        ncols = 1
        gs = GridSpec(nrows, ncols, figure=fig)
        ax = [None] * (nrows*ncols)

        ax[0] = fig.add_subplot(gs[0, 0])
        ax[1] = fig.add_subplot(gs[1, 0])
        ax[2] = fig.add_subplot(gs[2, 0])

        ax[0].plot(self.tspan[step_range], twist_object_vec_1[:, 0],
                   color='red', linewidth=2, label='v_x')
        ax[0].plot(self.tspan[step_range], twist_object_vec_1[:, 1],
                   color='green', linewidth=2, label='v_y')
        ax[0].plot(self.tspan[step_range], twist_object_vec_1[:, 2],
                   color='blue', linewidth=2, label='$\omega$')

        ax[1].plot(self.tspan[step_range], error_vec_1[:, 0],
                   color='red', linewidth=2, label='e_x')
        ax[1].plot(self.tspan[step_range], error_vec_1[:, 1],
                   color='green', linewidth=2, label='e_y')

        ax[2].plot(self.tspan[step_range], error_vec_2[:, 0],
                   color='red', linewidth=2, label='e_x')
        ax[2].plot(self.tspan[step_range], error_vec_2[:, 1],
                   color='green', linewidth=2, label='e_y')
        ax[2].plot(self.tspan[step_range], error_vec_2[:, 2],
                   color='blue', linewidth=2, label='e_$\\theta$')

        ax[0].legend(loc='upper left')
        ax[1].legend(loc='upper left')
        ax[2].legend(loc='upper left')

        ax[0].set_xlabel('time (s)')
        ax[1].set_xlabel('time (s)')
        ax[2].set_xlabel('time (s)')

        ax[0].set_title('object velocity')
        ax[1].set_title('qs factor 1 errors')
        ax[2].set_title('qs factor 2 errors')

        plt.show()
