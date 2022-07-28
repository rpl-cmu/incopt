from .Logger import Logger
import utils as utils

import pybullet as pb
import pybullet_data

import time
from datetime import datetime

import math
import numpy as np

import os

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

import pdb


class EnvKukaArmBlock():
    """
    A class to create a pybullet environment of a block being pushed by a fixed kuka arm.
    """

    def __init__(self, params, vis_flag=True):

        if vis_flag:
            pb.connect(pb.GUI)
            self.set_gui_params()
        else:
            pb.connect(pb.DIRECT)

        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        print(self.dir_path)

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

        # add pushing plane (slightly below robot arm base)
        self.plane_id = pb.loadURDF(
            "plane.urdf", [0, 0, -0.15], useFixedBase=True)

        # add kuka arm
        self.kuka_id = pb.loadURDF(
            "{}/../../models/arms/kuka_iiwa/kuka_with_endeff.urdf".format(self.dir_path), [0, 0, 0], useFixedBase=True)
        pb.resetBasePositionAndOrientation(
            self.kuka_id, [0, 0, 0], [0, 0, 0, 1])

        # add object being pushed
        self.obj_id = pb.loadURDF("{}/../../models/objects/block1.urdf".format(self.dir_path))
        pb.resetBasePositionAndOrientation(
            self.obj_id, self.init_obj_pos, self.init_obj_ori)
        self.verify_object_shape()

        # set gravity
        pb.setGravity(0, 0, -10)

        # set/get arm params
        self.kuka_ee_idx = 7
        self.num_joints = pb.getNumJoints(self.kuka_id)
        print("NUM JOINTS {} ".format(self.num_joints))
        self.joint_ids = [i for i in range(self.num_joints)]

        # joint damping coefficients
        self.jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

        # reset joint states to nominal pose (overrides physics simulation)
        self.reset_joint_states = [0, 0, 0, 0.5 *
                                   math.pi, 0, -math.pi * 0.5 * 0.66, 0, 0]
        for i in range(self.num_joints):
            pb.resetJointState(self.kuka_id, i, self.reset_joint_states[i])

        self.record_log_video = False
        if (self.record_log_video):
            self.log_id = pb.startStateLogging(
                pb.STATE_LOGGING_VIDEO_MP4, "{}/../../local/outputs/push_block_kuka.mp4".format(self.dir_path))

    def set_gui_params(self):

        enable_preview = False
        pb.configureDebugVisualizer(
            pb.COV_ENABLE_RGB_BUFFER_PREVIEW, enable_preview)
        pb.configureDebugVisualizer(
            pb.COV_ENABLE_DEPTH_BUFFER_PREVIEW, enable_preview)
        pb.configureDebugVisualizer(
            pb.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, enable_preview)
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, enable_preview)

        cam_tgt_pos = [-0.5, 0.32, -0.15]
        cam_dist = 1
        cam_yaw = 229
        cam_pitch = -36

        pb.resetDebugVisualizerCamera(
            cam_dist, cam_yaw, cam_pitch, cam_tgt_pos)

    def verify_object_shape(self):
        """" verify object properties in urdf match those in passed params """

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

    def get_logger(self):
        return self.logger

    def log_step(self, tstep):

        # get contact, link, obj information (A: self.kuka_id, B: self.obj_id)
        contact_info = pb.getContactPoints(self.kuka_id, self.obj_id)
        link_state = pb.getLinkState(self.kuka_id, self.kuka_ee_idx)
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

        # reset joint states to nominal pose (overrides physics simulation)
        for i in range(self.num_joints):
            pb.resetJointState(self.kuka_id, i, self.reset_joint_states[i])

        # reset block pose
        pb.resetBasePositionAndOrientation(
            self.obj_id, self.init_obj_pos, self.init_obj_ori)

    def step_sim(self):
        for i in range(1):
            pb.stepSimulation()

    def simulate(self, traj_vec):
        self.reset_sim()

        # I think this for loop is necessary to wait for the block to drop onto plane. Can this be done via initialization by setting params['block_size_z'] = -0.15? 
        for i in range(1000):
            pb.stepSimulation()

        for tstep in range(0, self.sim_length):

            pos_ee = [traj_vec[tstep, 0],
                      traj_vec[tstep, 1], 
                      traj_vec[tstep, 2]]

            # inverse kinematics
            joint_poses = pb.calculateInverseKinematics(
                self.kuka_id, self.kuka_ee_idx, pos_ee, self.init_ee_ori, jointDamping=self.jd)

            # motor control to follow IK solution
            for i in range(0, self.num_joints):
                pb.setJointMotorControl2(bodyIndex=self.kuka_id, jointIndex=i, controlMode=pb.POSITION_CONTROL,
                                         targetPosition=joint_poses[i], targetVelocity=0, force=500, positionGain=0.3, velocityGain=1)

            self.step_sim()

            self.log_step(tstep)
            # time.sleep(1. / 240.)

        if (self.record_log_video):
            pb.stopStateLogging(self.log_id)

    def simulate_reinitialize(self, traj_vec):
        """" sticky contacts test: re-initialize simulation with states from previous time step """

        self.reset_sim()
        reinit_obj_state = True
        reinit_ee_state = True

        for tstep in range(0, self.sim_length):

            # re-init object pose to value in previous time step
            if (reinit_obj_state):
                if (tstep == 0):
                    pb.resetBasePositionAndOrientation(
                        self.obj_id, self.init_obj_pos, self.init_obj_ori)
                else:
                    pb.resetBasePositionAndOrientation(
                        self.obj_id, self.logger.obj_pos[tstep-1, :], self.logger.obj_ori[tstep-1, :])

            # re-init arm joint states to exact IK solution
            pos_ee = [traj_vec[tstep, 0],
                      traj_vec[tstep, 1], traj_vec[tstep, 2]]
            joint_poses = pb.calculateInverseKinematics(
                self.kuka_id, self.kuka_ee_idx, pos_ee, self.init_ee_ori, jointDamping=self.jd)
            if (reinit_ee_state):
                for i in range(0, self.num_joints):
                    pb.resetJointState(self.kuka_id, i, joint_poses[i])
            else:
                for i in range(0, self.num_joints):
                    pb.setJointMotorControl2(bodyIndex=self.kuka_id, jointIndex=i, controlMode=pb.POSITION_CONTROL,
                                             targetPosition=joint_poses[i], targetVelocity=0, force=500, positionGain=0.3, velocityGain=1)

            self.step_sim()

            self.log_step(tstep)
            # time.sleep(1. / 240.)

        if (self.record_log_video):
            pb.stopStateLogging(self.log_id)
