import numpy as np


class ParamsParticleFilter:
    """ Parameters which specify particle filter

    """

    def __init__(self):
        self.tau = 1  # update timestep
        self.N = 2000  # number of particles
        self.threshold = 0.2 * self.N  # effective NoP threshold for resampling

        self.meas_dev = 2  # measurement deviation for update

        self.minPosEast = -400  # area to be flown over - east
        self.maxPosEast = 400
        self.minPosNorth = -100  # area to be flown over - north
        self.maxPosNorth = 400
        self.pos_noise = 2.0  # vehicle_position deviation for gaussian process

        self.strength_init_expectation = 1.5  # expectation for initial strength distribution
        self.strength_init_deviation = 1.0  # deviation for initial strength distribution
        self.strength_noise = 0.01  # strength deviation for gaussian process

        self.spread_init_expectation = 80  # expectation for initial spread distribution
        self.spread_init_deviation = 20  # deviation for initial spread distribution
        self.spread_noise = 0.1  # spread deviation for gaussian process

        self.kappa = .5

        # DBSCAN parameters
        self.cluster_epsilon = 60 # neighborhood parameter
        self.cluster_MinPts = 160
        self.cluster_maxParticles = 600
        self.cluster_colors = np.random.rand(10, 3)
