import numpy as np
import gtsam

def transform_to_frame2d(pt, frame_pose2d):
    """
    Transform 2D point (pt) from world frame to object frame (frame_pose2d)
    :param pt: point in world frame [2x1]
    :param frame_pose2d: object frame x,y,yaw [3x1]
    :return: point in object frame [2x1]
    """

    yaw = frame_pose2d[2, 0]
    R = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])
    t = -frame_pose2d[0:2]
    pt_tf = np.matmul(R, pt+t)

    return pt_tf

def transform_from_frame2d(pt, frame_pose2d):
    """
    Transform 2D point (pt) from object frame (frame_pose2d) to world frame
    :param pt: point in object frame [2x1]
    :param frame_pose2d: object frame x,y,yaw [3x1]
    :return: point in world frame [2x1]
    """

    yaw = frame_pose2d[2, 0]
    R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    t = frame_pose2d[0:2]
    pt_tf = np.matmul(R, pt)
    pt_tf = t + pt_tf
    
    return pt_tf

def rotate_to_frame2d(pt, frame_pose2d):
    """
    Rotate 2D point (pt) from world frame to object frame (frame_pose2d)
    :param pt: point in world frame [2x1]
    :param frame_pose2d: object frame x,y,yaw [3x1]
    :return: point in object frame [2x1]
    """

    yaw = frame_pose2d[2, 0]
    R = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])
    pt_tf = np.matmul(R, pt)

    return pt_tf

def cross2d(v1, v2):
    """
    Cross-product between two 2D vectors
    :param v1: vector 1
    :param v2: vector 2
    :return: v1 x v2
    """

    return v1[0]*v2[1] - v1[1]*v2[0]
