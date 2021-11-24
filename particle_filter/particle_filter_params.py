""" Parameter file for particle filter """


class ParamsParticleFilter:
    """ Parameters which specify particle filter

    tau: int
        Update timestep

    N: int
        Number of particles

    threshold: float
        Effective number of particles threshold for resampling

    meas_dev: int
        Measurement deviation for update

    minPosEast: int
        Border of area to be flown over in western direction

    maxPosEast: int
        Border of area to be flown over in eastern direction

    minPosNorth: int
        Border of area to be flown over in southern direction

    maxPosNorth: int
        Border of area to be flown over in northern direction

    pos_noise: float
        Vehicle_position deviation for gaussian process

    strength_init_expectation: float
        Expectation for initial strength distribution

    strength_init_deviation: float
        Deviation for initial strength distribution

    strength_noise: float
        Strength deviation for gaussian process

    spread_init_expectation: int
        Expectation for initial spread distribution

    spread_init_deviation: int
        Deviation for initial spread distribution

    spread_noise: float
        Spread deviation for gaussian process

    kappa: float
        Fraction for lateral particel prediction

    cluster_epsilon: int
        Neighborhood parameter for DBSCAN cluster algorithm

    cluster_MinPts: int
        Minimal points in cluster

    cluster_maxParticles: int
        Maximal points in cluster
    """

    def __init__(self):
        self.tau = 1
        self.N = 2000
        self.threshold = .1 * self.N  # effective NoP threshold for resampling

        self.meas_dev = 2

        self.minPosEast = -400
        self.maxPosEast = 400
        self.minPosNorth = -100
        self.maxPosNorth = 400
        self.pos_noise = 2.0

        self.strength_init_expectation = 1.5
        self.strength_init_deviation = 1.0
        self.strength_noise = 0.01

        self.spread_init_expectation = 80
        self.spread_init_deviation = 20
        self.spread_noise = 0.1

        self.kappa = .5

        # DBSCAN parameters
        self.cluster_epsilon = 60
        self.cluster_MinPts = 120
        self.cluster_maxParticles = 500
