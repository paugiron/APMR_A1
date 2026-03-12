#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

def plot_belief(belief):
    
    plt.figure()
    
    ax = plt.subplot(2,1,1)
    ax.matshow(belief.reshape(1, belief.shape[0]))
    ax.set_xticks(np.arange(0, belief.shape[0],1))
    ax.xaxis.set_ticks_position("bottom")
    ax.set_yticks([])
    ax.title.set_text("Grid")
    
    ax = plt.subplot(2,1,2)
    ax.bar(np.arange(0, belief.shape[0]), belief)
    ax.set_xticks(np.arange(0, belief.shape[0], 1))
    ax.set_ylim([0, 1.05])
    ax.title.set_text("Histogram")

    plt.show()
    

def motion_model(action, belief):
    n_cells = belief.shape[0]
    new_belief = np.zeros_like(belief)

    if action == 'F':
        correct_step = 1
        opposite_step = -1
    elif action == 'B':
        correct_step = -1
        opposite_step = 1
    else:
        raise ValueError("Action must be 'F' or 'B'.")

    for x_prev, prob in enumerate(belief):
        x_correct = x_prev + correct_step
        x_opposite = x_prev + opposite_step

        if 0 <= x_correct < n_cells:
            new_belief[x_correct] += 0.7 * prob
        else:
            new_belief[x_prev] += 0.7 * prob

        new_belief[x_prev] += 0.2 * prob

        if 0 <= x_opposite < n_cells:
            new_belief[x_opposite] += 0.1 * prob
        else:
            new_belief[x_prev] += 0.1 * prob

    return new_belief
  
    
def sensor_model(observation, belief, world):
    world = np.asarray(world).astype(int)

    if observation not in (0, 1):
        raise ValueError("Observation must be 0 (black) or 1 (white).")

    if observation == 1:
        likelihood = np.where(world == 1, 0.7, 0.1)
    else:
        likelihood = np.where(world == 0, 0.9, 0.3)

    updated_belief = belief * likelihood
    normalizer = np.sum(updated_belief)

    if normalizer <= 0.0:
        raise ValueError("Belief normalization failed: zero total probability.")

    return updated_belief / normalizer


def recursive_bayes_filter(actions, observations, belief, world):

    n_actions = len(actions)
    n_observations = len(observations)

    current_belief = belief.copy()

    if n_observations == n_actions + 1:
        current_belief = sensor_model(observations[0], current_belief, world)
        for action, observation in zip(actions, observations[1:]):
            current_belief = motion_model(action, current_belief)
            current_belief = sensor_model(observation, current_belief, world)

    return current_belief