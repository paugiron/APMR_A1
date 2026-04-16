#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

# icp_known_corresp: performs icp given that the input datasets
# are aligned so that Line1(:, QInd(k)) corresponds to Line2(:, PInd(k))
def icp_known_corresp(Line1, Line2, QInd, PInd):  
    Q = Line1[:, QInd]
    P = Line2[:, PInd]
    
    Mu1 = compute_mean(Q)
    Mu2 = compute_mean(P)    
    
    H = compute_H(Q, P, Mu1, Mu2)

    [R, t] = compute_R_t(H, Mu1, Mu2)
    
    # Compute the new positions of the points after
    # applying found rotation and translation to them
    NewLine = ...
    # YOUR CODE HERE
    raise NotImplementedError()
    # -----

    E = compute_error(Q, NewLine)
    
    return NewLine, E

    
# compute_mean: compute mean value for a [M x N] matrix
# consider a weight of 1 for each point
def compute_mean(M):
    
    # YOUR CODE HERE
    raise NotImplementedError()
    # -----


# compute_H: compute matrix H to use in SVD
# consider weight as 1
def compute_H(Q, P, MuQ, MuP):
    
    # YOUR CODE HERE
    raise NotImplementedError()
    # -----

    
# compute_R_t: compute rotation matrix and translation vector
# based on the SVD as presented in the lecture
def compute_R_t(H, MuQ, MuP):
    
    # YOUR CODE HERE
    raise NotImplementedError()
    # -----


# compute_error: compute the icp error
def compute_error(Q, OptimizedPoints):
    
    # YOUR CODE HERE
    raise NotImplementedError()
    # -----


# simply show the two lines
def show_figure(Line1, Line2):
    plt.figure()
    plt.scatter(Line1[0], Line1[1], marker='o', s=2, label='Line 1')
    plt.scatter(Line2[0], Line2[1], s=1, label='Line 2')
    
    plt.xlim([-8, 8])
    plt.ylim([-8, 8])
    plt.legend()  
    
    plt.show()
    

# initialize figure
def init_figure():
    fig = plt.gcf()
    fig.show()
    fig.canvas.draw()
    
    line1_fig = plt.scatter([], [], marker='o', s=2, label='Line 1')
    line2_fig = plt.scatter([], [], marker='o', s=1, label='Line 2')
    # plt.title(title)
    plt.xlim([-8, 8])
    plt.ylim([-8, 8])
    plt.legend()
    
    return fig, line1_fig, line2_fig


# update_figure: show the current state of the lines
def update_figure(fig, line1_fig, line2_fig, Line1, Line2, hold=False):
    line1_fig.set_offsets(Line1.T)
    line2_fig.set_offsets(Line2.T)
    if hold:
        plt.show()
    else:
        fig.canvas.flush_events()
        fig.canvas.draw()
        plt.pause(0.5)
