#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np


def draw2D(points):
    """Draw points on 2D

    Args:
        points (np.ndarray): in shape (N, 3)
    """
    pos_2d = points[:, :2]
    plt.scatter(pos_2d[:, 0], pos_2d[:, 1])
    plt.show()


def draw_foot(point1, point2, point3, point4):
    """Draw 4-foot positions, each point is np.ndarray, in shape (N, 3)"""
    time_steps = np.arange(len(point1))

    def plot_i(point, i, time_steps):
        plt.subplot(int(221 + i))
        plt.plot(time_steps, point)

    for i, point in enumerate([point1, point2, point3, point4]):
        plot_i(point, i, time_steps)

    plt.show()
