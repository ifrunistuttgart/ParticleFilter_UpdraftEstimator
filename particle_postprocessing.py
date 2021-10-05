""" This script is used for postprocessing the particle distribution from flight test data.
"""

import numpy as np
import scipy.io as sio
import particle_filter
import pandas as pd


def estimate_local_updraft(v_NED, V_A, sinkrate=0.674):
    """ Estimates local updraft.

    Parameters
    ----------
    v_NED : ndarray
        Ground speed of vehicle in NED-coordinates
    V_A : float
        Norm of air speed vector
    sinkrate : float
        Sel-sink of aircraft

    Returns
    -------
    updraft : ndarray
        Total energy compensated climb rate
    """

    mask = np.ones(10)/10  # moving average filter mask
    dt = 0.1
    dV_A = np.diff(V_A)/dt  # derivative of air speed
    dV_A_average = np.convolve(dV_A, mask, mode='same')

    w = - v_NED[1:, 2] + V_A[1:] * dV_A_average / 9.81

    updraft = w + sinkrate
    return updraft


""" Start Postprocessing """

# import data from csv-File
log_data = pd.read_csv('Flight_test_24_09/log_24-Sep-2021.csv')
vehicle_position = log_data[['x', 'y', 'z']].to_numpy()
vehicle_vel = log_data[['vx', 'vy', 'vz']].to_numpy()

# calculate local_updraft_estimate
airspeed = np.linalg.norm(vehicle_vel, axis=1)
local_updraft_estimate = estimate_local_updraft(vehicle_vel, airspeed)

# get number of filter steps and create arrays to store filter results
n_steps = local_updraft_estimate.size
filtered_state_array = np.zeros([4, 6, n_steps])
particle_array = np.zeros([5, 2000, n_steps])

# create filter instance
filter_instance = particle_filter.ParticleFilter(clustering_interval=1)

# run filter with vehicle_position and local_updraft_estimate reward
for i in range(n_steps):

    if i % 10 == 0:  # print out progress
        percentage = i/n_steps*100
        print("", end='\r')  # remove previous line
        print("Filter step {} of {} ({:.2f}%)".format(i, n_steps, percentage), end=' ')

    filter_instance.run_filter_step(vehicle_position=vehicle_position.T[:, i],
                                    local_updraft_estimate=local_updraft_estimate[i])
    # store filter step result to array
    particle_array[:, :, i] = filter_instance.particles
    filtered_state_array[:, :, i] = filter_instance.filtered_state

# export and save filter results
filter_data = {'particle_array': particle_array,
               'filtered_state_array': filtered_state_array, 'filter_steps': n_steps,
               'vehicle_position': vehicle_position, 'vehicle_velocity': vehicle_vel}

sio.savemat('./Flight_Test_24_09_filter_result.mat', filter_data)
