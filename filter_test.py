"""
    Simple test script for particle_filter module which uses logged flight data.
    Performs filtering and puts out a mat-File which stores particles and updraft positions
"""

import numpy as np
import scipy.io as sio
import particle_filter

# import data from mat-File
flight_data = sio.loadmat('./HIL_1705_filter_input.mat')
vehicle_position = flight_data['position_array']
local_updraft_estimate = flight_data['local_updraft_estimate'].flatten()

# create storage arrays for export
n_steps = local_updraft_estimate.size
print("Number of steps: {}".format(n_steps))
filtered_state_array = np.zeros([4, 6, n_steps])
particle_array = np.zeros([5, 2000, n_steps])

# create filter instance
filter_instance = particle_filter.ParticleFilter(clustering_interval=1)

# run filter steps with vehicle_position and local_updraft_estimate reward
for i in range(n_steps):
    if i % 10 == 0:
        print("Filter step {}".format(i))
    if i == 115:
        filter_instance.reset_filter()
    # run filter step
    filter_instance.run_filter_step(vehicle_position=vehicle_position[:, i],
                                    local_updraft_estimate=local_updraft_estimate[i])
    # store filter step result to array
    particle_array[:, :, i] = filter_instance.particles
    filtered_state_array[:, :, i] = filter_instance.filtered_state

# export and save filter results
filter_data = {'particle_array': particle_array,
               'filtered_state_array': filtered_state_array, 'filter_steps': n_steps}

sio.savemat('./HiL_1705_filter_result.mat', filter_data)

