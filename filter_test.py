"""
    Simple test script for particle_filter module which uses logged flight data.
    Performs filtering and puts out a mat-File which stores particles and updraft positions
"""

import numpy as np
import scipy.io as sio
#import particle_filter_accelerated
import particle_filter
import cProfile
import pstats

# import data from mat-File
flight_data = sio.loadmat('./filter_result_matlab.mat')
vehicle_position = flight_data['position_array']
local_updraft_estimate = flight_data['energy_reward_array'].flatten()

# create storage arrays for export
n_steps = local_updraft_estimate.size
filtered_state_array = np.zeros([4, 6, n_steps])
particle_array = np.zeros([5, 2000, n_steps])

# create filter instance
filter_instance = particle_filter.ParticleFilter(clustering_interval=10)
#filter_instance = particle_filter_accelerated.ParticleFilter(clustering_interval=1)
# create profiler instance and start profiling
p = cProfile.Profile()
p.enable()

# run filter steps with vehicle_position and local_updraft_estimate reward
for i in range(n_steps):
    if i % 10 == 0:
        print("Filter step {}".format(i))
    filter_instance.run_filter_step(vehicle_position=vehicle_position[:, i],
                                    local_updraft_estimate=local_updraft_estimate[i])
    particle_array[:, :, i] = filter_instance.particles
    filtered_state_array[:, :, i] = filter_instance.filtered_state
p.disable()
# export and save filter results
filter_data = {'particle_array': particle_array, 'position_array': vehicle_position,
               'filtered_state_array': filtered_state_array}

sio.savemat('./pf_python_sklearn.mat', filter_data)

stats = pstats.Stats(p).sort_stats('cumtime')
stats.print_stats()
