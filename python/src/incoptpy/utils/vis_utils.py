#!/usr/bin/env python

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.collections import PatchCollection
import matplotlib.patches as mpatches
import matplotlib.lines as mlines

plt.rcParams.update({'font.size': 16})

def draw_endeff(poses_ee, color="dimgray", label=None, ax=None):

    eff_radius = 0.04
    plt.plot(poses_ee[-1][0], poses_ee[-1][1],
             'k*') if ax is None else ax.plot(poses_ee[-1][0], poses_ee[-1][1], 'k*')
    ori = poses_ee[-1, 2]

    circ = mpatches.Circle((poses_ee[-1][0], poses_ee[-1][1]),
                           eff_radius, facecolor='None', edgecolor='grey',
                           linewidth=2, linestyle='-')
    plt.gca().add_patch(circ)

    sz_arw = 0.03
    (dx, dy) = (sz_arw * -math.sin(ori), sz_arw * math.cos(ori))
    plt.arrow(poses_ee[-1, 0], poses_ee[-1, 1], dx, dy, linewidth=2,
              head_width=0.001, color=color, head_length=0.01, fc='dimgray', ec='dimgray') if ax is None else ax.arrow(poses_ee[-1, 0], poses_ee[-1, 1], dx, dy, linewidth=2,
                                                                                                                       head_width=0.001, color=color, head_length=0.01, fc='dimgray', ec='dimgray')

    ee_radius = 0.0075
    circle = mpatches.Circle(
        (poses_ee[-1][0], poses_ee[-1][1]), color='dimgray', radius=ee_radius)
    plt.gca().add_patch(circle) if ax is None else ax.add_patch(circle)


def draw_object(poses_obj, shape="disc", color="dimgray", label=None, ax=None):

    linestyle_gt = '--' if (color == "dimgray") else '-'
    plt.plot(poses_obj[:, 0], poses_obj[:, 1], color=color,
             linestyle=linestyle_gt, label=label, linewidth=2, alpha=0.9) if ax is None else ax.plot(poses_obj[:, 0], poses_obj[:, 1], color=color,
                                                                                                     linestyle=linestyle_gt, label=label, linewidth=2, alpha=0.9)

    if (shape == "disc"):
        disc_radius = 0.088
        circ_obj = mpatches.Circle((poses_obj[-1][0], poses_obj[-1][1]), disc_radius,
                                   facecolor='None', edgecolor=color, linestyle=linestyle_gt, linewidth=2, alpha=0.9)
        plt.gca().add_patch(circ_obj) if ax is None else ax.add_patch(circ_obj)

        # cross-bars
        (x0, y0, yaw) = (poses_obj[-1][0],
                         poses_obj[-1][1], poses_obj[-1][2])
        r = disc_radius
        plt.plot([x0 + r * math.cos(yaw), x0 - r * math.cos(yaw)],
                 [y0 + r * math.sin(yaw), y0 - r * math.sin(yaw)],
                 linestyle=linestyle_gt, color=color, alpha=0.4) if ax is None else ax.plot([x0 + r * math.cos(yaw), x0 - r * math.cos(yaw)],
                                                                                            [y0 + r * math.sin(
                                                                                                yaw), y0 - r * math.sin(yaw)],
                                                                                            linestyle=linestyle_gt, color=color, alpha=0.4)
        plt.plot([x0 - r * math.sin(yaw), x0 + r * math.sin(yaw)],
                 [y0 + r * math.cos(yaw), y0 - r * math.cos(yaw)],
                 linestyle=linestyle_gt, color=color, alpha=0.4) if ax is None else ax.plot([x0 - r * math.sin(yaw), x0 + r * math.sin(yaw)],
                                                                                            [y0 + r * math.cos(
                                                                                                yaw), y0 - r * math.cos(yaw)],
                                                                                            linestyle=linestyle_gt, color=color, alpha=0.4)

    elif (shape == "rect"):
        rect_len_x = 0.15
        rect_len_y = 0.4

        yaw = poses_obj[-1][2]
        R = np.array([[np.cos(yaw), -np.sin(yaw)],
                      [np.sin(yaw), np.cos(yaw)]])
        offset = np.matmul(R, np.array(
            [[0.5*rect_len_x], [0.5*rect_len_y]]))
        xb = poses_obj[-1][0] - offset[0]
        yb = poses_obj[-1][1] - offset[1]
        rect = mpatches.Rectangle((xb, yb), rect_len_x, rect_len_y, angle=(
            np.rad2deg(yaw)), facecolor='None', edgecolor=color, linestyle=linestyle_gt, linewidth=2)
        plt.gca().add_patch(rect) if ax is None else ax.add_patch(rect)

    elif (shape == "ellip"):
        ellip_len_x = 0.1638
        ellip_len_y = 0.2428

        xb = poses_obj[-1][0]
        yb = poses_obj[-1][1]
        yaw = poses_obj[-1][2]
        ellip = mpatches.Ellipse((xb, yb), ellip_len_x, ellip_len_y, angle=(
            np.rad2deg(yaw)), facecolor='None', edgecolor=color, linestyle=linestyle_gt, linewidth=2)
        plt.gca().add_patch(ellip) if ax is None else ax.add_patch(ellip)
