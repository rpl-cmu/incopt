from .Logger import Logger
import utils as utils

import pybullet as pb
import pybullet_data

import time
from datetime import datetime

import math
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

import pdb


class EnvFloatingArmBlock():
    """
    A class to create a pybullet environment of a block being pushed by a floating arm/gripper.
    """

    def __init__(self, params, vis_flag=True):

        if vis_flag:
            pb.connect(pb.GUI)
            self.set_gui_params()
        else:
            pb.connect(pb.DIRECT)

        self.debug = True

        # read in sim params
        self.params = params

        self.tstart = params['tstart']
        self.tend = params['tend']
        self.dt = params['dt']
        self.sim_length = np.int((self.tend - self.tstart) / self.dt)

        self.init_ee_pos = params['init_ee_pos']
        self.init_ee_ori = params['init_ee_ori']
        self.init_obj_pos = params['init_obj_pos']
        self.init_obj_ori = params['init_obj_ori']

        self.logger = Logger(params)

        # set additional path to access bullet lib models
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())

        # add pushing plane
        self.plane_id = pb.loadURDF(
            "plane.urdf", [0, 0, 0.0], useFixedBase=True)

        # add floating arm
        self.arm_id = pb.loadURDF(
            "../models/arms/ball_arm.urdf", [0, 0, 0], globalScaling=1.0, useFixedBase=False)
        pb.resetBasePositionAndOrientation(
            self.arm_id, [0, 0, 0], [0, 0, 0, 1])

        self.arm_ee_idx = 0
        self.num_joints = pb.getNumJoints(self.arm_id)

        self.constraint_id = pb.createConstraint(
            self.arm_id, 0, -1, -1, pb.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 1])

        # add object being pushed
        self.obj_id = pb.loadURDF("../models/objects/block2.urdf")
        pb.resetBasePositionAndOrientation(
            self.obj_id, self.init_obj_pos, self.init_obj_ori)
        self.verify_object_shape()

        # set gravity
        pb.setGravity(0, 0, -10)

        # record video log
        self.record_log_video = False
        if (self.record_log_video):
            self.log_id = pb.startStateLogging(
                pb.STATE_LOGGING_VIDEO_MP4, "../local/outputs/push_floating_block.mp4")

    def get_logger(self):
        return self.logger

    def set_gui_params(self):

        enable_preview = False
        pb.configureDebugVisualizer(
            pb.COV_ENABLE_RGB_BUFFER_PREVIEW, enable_preview)
        pb.configureDebugVisualizer(
            pb.COV_ENABLE_DEPTH_BUFFER_PREVIEW, enable_preview)
        pb.configureDebugVisualizer(
            pb.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, enable_preview)
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, enable_preview)

        cam_tgt_pos = [-0.17, 0.52, 0.53]
        cam_dist = 2
        cam_yaw = 270
        cam_pitch = -80

        pb.resetDebugVisualizerCamera(
            cam_dist, cam_yaw, cam_pitch, cam_tgt_pos)

    def verify_object_shape(self):
        """" Verify object properties in urdf match those in passed params """

        visual_data = pb.getVisualShapeData(self.obj_id, -1)
        collision_data = pb.getCollisionShapeData(self.obj_id, -1)

        visual_dims = visual_data[0][3]
        collision_dims = collision_data[0][3]

        assert (self.params['block_size_x'] == visual_dims[0]
                == collision_dims[0]), "object x-dim mismatch."
        assert (self.params['block_size_y'] == visual_dims[1]
                == collision_dims[1]), "object y-dim mismatch."
        assert (self.params['block_size_z'] == visual_dims[2]
                == collision_dims[2]), "object z-dim mismatch."

    def log_step(self, tstep):

         # get contact, endeff, obj information (A: self.arm_id, B: self.obj_id)
        contact_info = pb.getContactPoints(self.arm_id, self.obj_id)
        link_state = pb.getLinkState(self.arm_id, self.arm_ee_idx)
        obj_pose = pb.getBasePositionAndOrientation(self.obj_id)

        # store in logger object
        self.logger.t[tstep, :] = self.tstart + tstep * self.dt

        self.logger.ee_pos[tstep, :] = link_state[0]
        self.logger.ee_ori[tstep, :] = link_state[1]
        self.logger.ee_ori_mat[tstep, :] = pb.getMatrixFromQuaternion(
            link_state[1])  # row-major order
        self.logger.ee_ori_rpy[tstep, :] = pb.getEulerFromQuaternion(
            self.logger.ee_ori[tstep, :])

        self.logger.obj_pos[tstep, :] = obj_pose[0]
        self.logger.obj_ori[tstep, :] = obj_pose[1]
        self.logger.obj_ori_mat[tstep, :] = pb.getMatrixFromQuaternion(
            obj_pose[1])  # row-major order
        self.logger.obj_ori_rpy[tstep, :] = pb.getEulerFromQuaternion(
            self.logger.obj_ori[tstep, :])

        if (len(contact_info) > 0):
            self.logger.contact_flag[tstep, :] = 1
            self.logger.contact_pos_onA[tstep, :] = contact_info[0][5]
            self.logger.contact_pos_onB[tstep, :] = contact_info[0][6]
            self.logger.contact_normal_onB[tstep, :] = contact_info[0][7]
            self.logger.contact_distance[tstep, :] = contact_info[0][8]
            self.logger.contact_normal_force[tstep, :] = contact_info[0][9]

            self.logger.lateral_friction1[tstep, :] = contact_info[0][10]
            self.logger.lateral_frictiondir1[tstep, :] = contact_info[0][11]
            self.logger.lateral_friction2[tstep, :] = contact_info[0][12]
            self.logger.lateral_frictiondir2[tstep, :] = contact_info[0][13]

    def reset_sim(self):

        # reset arm state
        pb.changeConstraint(self.constraint_id,
                            self.init_ee_pos, self.init_ee_ori)

        # reset block pose
        pb.resetBasePositionAndOrientation(
            self.obj_id, self.init_obj_pos, self.init_obj_ori)

    def step_sim(self):
        for i in range(1):
            pb.stepSimulation()

    def draw_trajectories(self, tstep):

        obj_pos_prev = self.logger.obj_pos[tstep-1, :]
        obj_pos_curr = self.logger.obj_pos[tstep, :]

        ee_pos_prev = self.logger.ee_pos[tstep-1, :]
        ee_pos_curr = self.logger.ee_pos[tstep, :]

        line_ee = pb.addUserDebugLine(
            ee_pos_prev, ee_pos_curr, lineColorRGB=(0, 1, 0), lineWidth=2)
        line_obj = pb.addUserDebugLine(
            obj_pos_prev, obj_pos_curr, lineColorRGB=(1, 0, 0), lineWidth=3)

        return (line_ee, line_obj)

    def push_const_vel__object(self):
        """" Constant velocity pushing in object frame """

        obj_pos, obj_ori = pb.getBasePositionAndOrientation(self.obj_id)
        link_state = pb.getLinkState(self.arm_id, self.arm_ee_idx)
        ee_pos = link_state[0]
        ee_ori = link_state[1]

        obj_yaw = pb.getEulerFromQuaternion(obj_ori)[2]
        ee_yaw = pb.getEulerFromQuaternion(ee_ori)[2]

        vel = [8, 0, 0]  # vx, vy, omega

        # transform ee_pos from world to object frame
        ee_pos__world = np.array([[ee_pos[0]], [ee_pos[1]]])
        frame_pose2d = np.array([[obj_pos[0]], [obj_pos[1]], [obj_yaw]])
        ee_pos__object = utils.transform_to_frame2d(
            ee_pos__world, frame_pose2d)

        # Calculate new arm endeff position
        ee_new_pos__object = np.array(
            [ee_pos__object[0] + self.dt * vel[0], ee_pos__object[1] + self.dt * vel[1]])
        ee_new_pos__world = utils.transform_from_frame2d(
            ee_new_pos__object, frame_pose2d)
        ee_new_z = 0.5 * self.params['block_size_z']

        # Calculate new arm endeff orientation
        ee_init_yaw = pb.getEulerFromQuaternion(self.init_ee_ori)[2]
        ee_new_yaw = ee_init_yaw + obj_yaw
        # ee_new_yaw = ee_yaw + self.dt * vel[2]

        ee_new_pos = [ee_new_pos__world[0, 0],
                      ee_new_pos__world[1, 0], ee_new_z]
        ee_new_ori = [0, 0, ee_new_yaw]
        return (ee_new_pos, ee_new_ori)

    def push_obj_dir(self):
        """" Push along object direction """

        obj_pos, obj_ori = pb.getBasePositionAndOrientation(self.obj_id)
        link_state = pb.getLinkState(self.arm_id, self.arm_ee_idx)
        ee_pos = link_state[0]
        ee_ori = link_state[1]

        obj_yaw = pb.getEulerFromQuaternion(obj_ori)[2]
        ee_yaw = pb.getEulerFromQuaternion(ee_ori)[2]

        dist = 4e-2

        # Calculate new arm endeff position
        ee_new_x = ee_pos[0] + dist * math.cos(obj_yaw)
        ee_new_y = ee_pos[1] + dist * math.sin(obj_yaw)
        ee_new_z = 0.5 * self.params['block_size_z']

        # Calculate new arm endeff orientation
        ee_init_yaw = pb.getEulerFromQuaternion(self.init_ee_ori)[2]
        vec = np.array([obj_pos[0] - ee_new_x,
                        obj_pos[1] - ee_new_y, 0])
        vec = vec / np.linalg.norm(vec)
        ee_new_yaw = ee_init_yaw + math.atan2(vec[0], -vec[1])
        ee_new_yaw = (ee_new_yaw - np.pi/2) % (2 * np.pi) + 0.1

        ee_new_pos = [ee_new_x, ee_new_y, ee_new_z]
        ee_new_ori = [0, 0, ee_new_yaw]
        return (ee_new_pos, ee_new_ori)

    def simulate(self):

        self.reset_sim()
        for i in range(1000):
            pb.stepSimulation()

        for tstep in range(0, self.sim_length):

            # print("Simulation time step: {0}".format(tstep))

            ee_new_pos, ee_new_ori = self.push_const_vel__object()
            # ee_new_pos, ee_new_ori = self.push_obj_dir()

            # Move arm to new position/orientation
            pb.changeConstraint(self.constraint_id, ee_new_pos,
                                pb.getQuaternionFromEuler(ee_new_ori))

            self.step_sim()
            self.log_step(tstep)

            # time.sleep(1. / 240.)

            # Draw desired trajectory
            if(self.debug):
                (line_ee, line_obj) = self.draw_trajectories(tstep)

        if (self.record_log_video):
            pb.stopStateLogging(self.log_id)
