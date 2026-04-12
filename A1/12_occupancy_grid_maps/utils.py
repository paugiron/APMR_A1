#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import bresenham as bh

def plot_gridmap(gridmap):
    gridmap = np.array(gridmap, dtype=np.float64)
    plt.figure()
    plt.imshow(gridmap, cmap='Greys',vmin=0, vmax=1)
    plt.show()
    
    
def init_gridmap(size, res):
    gridmap = np.zeros([int(np.ceil(size/res)), int(np.ceil(size/res))])
    return gridmap


def world2map(pose, gridmap, map_res):
    origin = np.array(gridmap.shape)/2
    new_pose = np.zeros((pose.shape))
    new_pose[0:] = np.round(pose[0:]/map_res) + origin[0]
    new_pose[1:] = np.round(pose[1:]/map_res) + origin[1]
    return new_pose.astype(int)


def v2t(pose):
    c = np.cos(pose[2])
    s = np.sin(pose[2])
    tr = np.array([[c, -s, pose[0]], [s, c, pose[1]], [0, 0, 1]])
    return tr    


def ranges2points(ranges):
    # laser properties
    start_angle = -1.5708
    angular_res = 0.0087270
    max_range = 30
    # rays within range
    num_beams = ranges.shape[0]
    idx = (ranges < max_range) & (ranges > 0)
    # 2D points
    angles = np.linspace(start_angle, start_angle + (num_beams*angular_res), num_beams)[idx]
    points = np.array([np.multiply(ranges[idx], np.cos(angles)), np.multiply(ranges[idx], np.sin(angles))])
    # homogeneous points
    points_hom = np.append(points, np.ones((1, points.shape[1])), axis=0)
    return points_hom


def ranges2cells(r_ranges, w_pose, gridmap, map_res):
    # ranges to points
    r_points = ranges2points(r_ranges)
    w_P = v2t(w_pose)
    w_points = np.matmul(w_P, r_points)
    # covert to map frame
    m_points = world2map(w_points, gridmap, map_res)
    m_points = m_points[0:2,:]
    return m_points


def poses2cells(w_pose, gridmap, map_res):
    # covert to map frame
    m_pose = world2map(w_pose, gridmap, map_res)
    return m_pose  


def bresenham(x0, y0, x1, y1):
    l = np.array(list(bh.bresenham(x0, y0, x1, y1)))
    return l

    
def prob2logodds(p):
    # YOUR CODE HERE
    # convert occupancy probability to log-odds
    # l = log( p / (1 - p) )
    p = np.asarray(p, dtype=np.float64)
    # Numerical safety: avoid p=0 or p=1 (would produce infinite log-odds)
    p = np.clip(p, 1e-6, 1 - 1e-6)
    return np.log(p / (1 - p))
    # -----

    
def logodds2prob(l):
    # YOUR CODE HERE
    # inverse conversion from log-odds to probability
    # p = 1 / (1 + exp(-l))
    l = np.asarray(l, dtype=np.float64)
    return 1.0 / (1.0 + np.exp(-l))
    # -----

    
def inv_sensor_model(cell, endpoint, prob_occ, prob_free):
    # YOUR CODE HERE
    """Build the inverse sensor model for one laser beam.

    Output format per row: [x_cell, y_cell, occupancy_probability].
    - Cells on the ray before the hit -> prob_free
    - Final hit cell (endpoint)       -> prob_occ
    """
    x0, y0 = int(cell[0]), int(cell[1])
    x1, y1 = int(endpoint[0]), int(endpoint[1])

    # compute all cells in the beam using Bresenham line tracing
    ray_cells = bresenham(x0, y0, x1, y1)
    sensor_model = np.zeros((ray_cells.shape[0], 3), dtype=np.float64)
    sensor_model[:, 0:2] = ray_cells
    # By default, traversed cells are considered free.
    sensor_model[:, 2] = prob_free
    # The final cell corresponds to the obstacle hit by the laser.
    sensor_model[-1, 2] = prob_occ

    return sensor_model
    # -----


def grid_mapping_with_known_poses(ranges_raw, poses_raw, occ_gridmap, map_res, prob_occ, prob_free, prior):
    # YOUR CODE HERE
    """Compute occupancy grid map using known robot poses and laser scans.

    Big picture:
    1) Convert map to log-odds.
    2) For each pose, project all laser endpoints into map cells.
    3) For each beam, update traversed cells (free) and endpoint (occupied).
    4) Convert final log-odds map back to probabilities.
    """
    # We keep the map in log-odds because updates are additive.
    log_odds_map = prob2logodds(occ_gridmap.copy())
    prior_log_odds = prob2logodds(prior)

    rows, cols = occ_gridmap.shape

    # Loop over time: one robot pose + one full scan each iteration.
    for time_idx in range(poses_raw.shape[0]):
        # Robot pose in map coordinates (cell indices).
        m_robot_pose = poses2cells(poses_raw[time_idx, :], occ_gridmap, map_res)
        # Laser hits (beam endpoints) converted to map cells.
        m_endpoints = ranges2cells(ranges_raw[time_idx, :], poses_raw[time_idx, :], occ_gridmap, map_res)

        # We only need x,y cell indices (theta is not needed for ray casting).
        robot_cell = m_robot_pose[0:2]

        # Process each laser beam independently.
        for beam_idx in range(m_endpoints.shape[1]):
            endpoint = m_endpoints[:, beam_idx]
            # Build local inverse model for this single beam.
            sensor_cells = inv_sensor_model(robot_cell, endpoint, prob_occ, prob_free)

            # Update every affected cell using:
            # l_t(m_i) = l_{t-1}(m_i) + l(m_i|z_t) - l0
            # where l0 is the prior log-odds.
            for sensor_cell in sensor_cells:
                map_x = int(sensor_cell[0])
                map_y = int(sensor_cell[1])

                # Safety check: skip cells that fall outside the map array.
                if 0 <= map_x < rows and 0 <= map_y < cols:
                    log_odds_map[map_x, map_y] += prob2logodds(sensor_cell[2]) - prior_log_odds

    # Return standard occupancy probabilities for plotting/inspection.
    return logodds2prob(log_odds_map)
    # -----