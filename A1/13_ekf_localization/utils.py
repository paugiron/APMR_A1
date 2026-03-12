# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import matplotlib.transforms as transforms
import numpy as np
from matplotlib.patches import Ellipse
from math import sin, cos, atan2, sqrt

def plot_state(mu, S, M):

    plt.figure()

    # Initialize figure
    ax = plt.gca()
    ax.set_xlim([np.min(M[:, 0]) - 2, np.max(M[:, 0]) + 2])
    ax.set_xlim([np.min(M[:, 1]) - 2, np.max(M[:, 1]) + 2])
    plt.plot(M[:, 0], M[:, 1], '^r')
    plt.title('EKF Localization')

    # visualize result
    plt.plot(mu[0], mu[1], '.b')
    plot_2dcov(mu, S)
    plt.draw()
    #plt.pause(0.01)

    plt.show()


def plot_2dcov(mu, cov):

    # Covariance only in x,y
    d, v = np.linalg.eig(cov[:-1, :-1])

    # Ellipse orientation
    a = np.sqrt(d[0])
    b = np.sqrt(d[1])

    # Compute ellipse orientation
    if (v[0, 0] == 0):
        theta = np.pi / 2
    else:
        theta = np.arctan2(v[0, 1], v[0, 0])

    # Create an ellipse
    ellipse = Ellipse((mu[0], mu[1]),
                      width=a * 2,
                      height=b * 2,
                      angle=np.deg2rad(theta),
                      edgecolor='blue',
                      alpha=0.3)

    ax = plt.gca()

    return ax.add_patch(ellipse)


def wrapToPi(theta):
    while theta < -np.pi:
        theta = theta + 2 * np.pi
    while theta > np.pi:
        theta = theta - 2 * np.pi
    return theta


def inv_motion_model(u_t):
    trans = sqrt((u_t[1][0]-u_t[0][0])**2 + (u_t[1][1]-u_t[0][1])**2)
    rot1  = wrapToPi(atan2((u_t[1][1]-u_t[0][1]),(u_t[1][0]-u_t[0][0])) - u_t[0][2])
    rot2  = wrapToPi(u_t[1][2] - u_t[0][2] - rot1)

    return rot1, trans, rot2


def ekf_predict(mu, sigma, u_t, R_t):
    # Complete the following code according to the indicated steps 
    
    # Estimate the deltas in odometry, given u_t (d_rot1, dtrans, d_rot2)
    # YOUR CODE HERE
    raise NotImplementedError()
    # -----

    # Compute the Jacobian of the motion model with respect to the previous state (G_t)
    # YOUR CODE HERE
    raise NotImplementedError()
    # -----

    # Compute the Jacobian of the motion model with respect to the motion parameters (V_t)
    # YOUR CODE HERE
    raise NotImplementedError()
    # -----
    
    # Compute the prediction of the mean (mu_bar)
    # YOUR CODE HERE
    raise NotImplementedError()
    # -----
             
    # Compute the prediction of the covariance matrix (sigma_bar)
    # YOUR CODE HERE
    raise NotImplementedError()
    # -----    


def ekf_correct(mu_bar, sigma_bar, z, Q, M):
    
    # Complete the following code according to the indicated steps
    for i in range(z.shape[1]):
        
        # Get the id of the observed landmark
        j = int(z[2,i])
        
        # Get the coordinates of the corresponding landmark
        lx = M[j,0]
        ly = M[j,1]
        
        # Compute the distance between the pose of the robot and the landmark (q and dist = sqrt(q))
        # YOUR CODE HERE
        raise NotImplementedError()
        # -----
        
        # Compute the Jacobian for the observation model (H^i_t)
        # YOUR CODE HERE
        raise NotImplementedError()
        # -----
    
        # Compute the Kalman Gain (K)
        # YOUR CODE HERE
        raise NotImplementedError()
        # -----
        
        # Compute the expected observation (z_hat)
        # YOUR CODE HERE
        raise NotImplementedError()
        # -----
    
        # Correct the mean (mu)
        # YOUR CODE HERE
        raise NotImplementedError()
        # -----
    
        # Correct the covariance matrix (sigma)
        # YOUR CODE HERE
        raise NotImplementedError()
        # -----