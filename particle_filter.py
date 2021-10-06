import numpy as np
import scipy.spatial
import particle_filter_params
from sklearn.cluster import DBSCAN


class ParticleFilter:
    """ Particle filter object

    Attributes
    ----------

    params : ParamsParticleFilter
        Contains all parameters for particle filter

    particles : numpy-array
        Contains particles with North-East vehicle_position and weight

    IDX : numpy-array
        Vector which contains index of assigned cluster for each particle

    prevent : boolean
        Flag which prevents clustered particles from being resampled

    filter_step : int
        Number of performed filter steps

    clustering_interval : int
        Steps between execution of clustering

    cluster_num : int
        Number of detected clusters

    filtered_state : numpy-array
        NE-Position of detected updrafts
    """

    def __init__(self, clustering_interval=1):
        """
        Parameters
        ----------
        clustering_interval : Steps between clustering
        """

        self.params = particle_filter_params.ParamsParticleFilter()
        self.prevent = True
        self.particles = self.init_particles()
        self.IDX = np.zeros(self.params.N, dtype=np.uint64)
        self.cluster_num = 0
        self.clustering_interval = clustering_interval
        self.filtered_state = np.zeros([4, 6])
        self.filter_step = 0
        self.test = 0

    def reset_filter(self):
        self.particles = self.init_particles()
        self.IDX = np.zeros(self.params.N, dtype=np.uint64)
        self.cluster_num = 0
        self.filtered_state = np.zeros([4, 6])

    def init_particles(self):
        """ Initializes particles with random NE-positions and equal weights

        Returns
        -------
        particles: numpy-array
        """
        particles = np.zeros([5, self.params.N])

        for i in range(self.params.N):
            particles[:, i] = self.generate_particle()

        return particles

    def generate_particle(self):
        """ Generates one particle

        Returns
        -------
        particle: numpy-array
        """
        particle = np.zeros([5, ])
        particle[0] = np.round(self.params.minPosNorth +
                               (self.params.maxPosNorth - self.params.minPosNorth) * np.random.rand())
        particle[1] = np.round(self.params.minPosEast +
                               (self.params.maxPosEast - self.params.minPosEast) * np.random.rand())
        particle[2] = np.random.normal(self.params.strength_init_expectation, self.params.strength_init_deviation)
        particle[3] = np.random.normal(self.params.spread_init_expectation, self.params.spread_init_deviation)
        particle[4] = 1 / self.params.N

        return particle

    def run_filter_step(self, vehicle_position: np.ndarray, local_updraft_estimate):
        """ Performs one step of particle filter

        Parameters
        ----------
        vehicle_position : 3x1 numpy array
            Position of vehicle in NED coordinates
        local_updraft_estimate: float
            Estimated total local_updraft_estimate compensated climb rate

        Returns
        -------
        pf_updraft_estimates: numpy-array
            Positions, strengths, and spreads of estimated updrafts
        updraft_detected: boolean
            Boolean flag, if at least one updraft was detected
        """

        # run filter steps
        self.filter_step += 1
        self.prediction_step(vehicle_position, local_updraft_estimate)
        self.resampling()

        # run clustering every "clustering_interval" seconds
        if (self.filter_step % self.clustering_interval) == 0:
            self.cluster_analysis()
            self.density_limitation()
            self.calculate_filtered_state()

        # assign return values
        pf_updraft_estimates = self.filtered_state

        if np.count_nonzero(self.filtered_state) == 0:
            updraft_detected = 0
        else:
            updraft_detected = 1

        return pf_updraft_estimates, updraft_detected

    def prediction_step(self, vehicle_position: np.ndarray, local_updraft_estimate):
        """ Performs prediction and reweighting step

        Parameters
        ----------
        vehicle_position : 3x1 numpy array
            Position in NED coordinates
        local_updraft_estimate : float
            Estimated total local_updraft_estimate compensated climb rate

        """

        # Randomly move particles in NE direction
        self.particles[0:2, :] = (self.particles[0:2, :] + np.random.randn(2, self.params.N) *
                                  self.params.tau * self.params.kappa * self.params.pos_noise)

        self.particles[2, :] = self.particles[2, :] + np.random.randn(self.params.N) * self.params.strength_noise
        self.particles[3, :] = self.particles[3, :] + np.random.randn(self.params.N) * self.params.spread_noise

        # calculate squared Euclidean distance between particles and vehicle_position
        delta_pos = np.power(np.linalg.norm(vehicle_position[0:2].reshape(2, 1) - self.particles[0:2, :], axis=0), 2)

        # calculate and perform update vector
        update = (self.particles[2, :] *
                  np.exp(-delta_pos / (2 * self.particles[3, :] ** 2)))

        self.particles[4, :] = (self.particles[4, :] * (1 / (self.params.meas_dev * np.sqrt(2 * np.pi))) *
                                np.exp(-((local_updraft_estimate - update) / self.params.meas_dev) ** 2 / 2))

        # normalize weights
        weight_sum = np.sum(self.particles[4, :])
        self.particles[4, :] = self.particles[4, :] / weight_sum

    def resampling(self):
        """ Performs resampling, if effective number of particles falls below a given threshold

        """

        # calculate number of effective particles
        denominator = np.sum(self.particles[4, :] ** 2)
        n_eff = 1 / denominator

        if n_eff < self.params.threshold:
            particles_new = np.zeros([5, self.params.N])
            rho = np.random.rand() / self.params.N
            cum_sum = self.particles[4, 0]
            j = 0

            for i in range(self.params.N):
                u = rho + i / self.params.N

                while u > cum_sum:
                    j = j + 1
                    cum_sum = cum_sum + self.particles[4, j]
                particles_new[:, i] = self.particles[:, j]

            # prevents clustered particles from being resampled
            if self.prevent:

                self.particles[:4, self.IDX == 0] = particles_new[:4, self.IDX == 0]
                clustered_particles = self.particles[:, self.IDX != 0]
                clustered_particles_new = particles_new[:, self.IDX != 0]

                for j in range(np.count_nonzero(self.IDX)):

                    if np.random.rand() <= 0.1:
                        clustered_particles[:4, j] = clustered_particles_new[:4, j]

                self.particles[:4, self.IDX != 0] = clustered_particles[:4, :]
            else:
                self.particles[:4, :] = particles_new[:4, :]

            self.particles[4, :] = 1 / self.params.N

    def cluster_analysis(self):
        """ Use DBSCAN from sklearn to cluster particles

        """

        clustering = DBSCAN(eps=self.params.cluster_epsilon,
                            min_samples=self.params.cluster_MinPts).fit(self.particles[:2, :].T)

        self.IDX = clustering.labels_ + 1
        self.cluster_num = len(set(self.IDX)) - (1 if 0 in self.IDX else 0)

    def density_limitation(self):
        """ Randomly redistributes excess points in clusters, as each cluster has a maximum number of points

        """

        limited_set = self.particles

        for c in range(np.max(self.IDX)):  # if max is 0 no code executed
            index_vector = np.where(self.IDX == (c + 1))
            cardinality = np.size(index_vector)

            if cardinality > self.params.cluster_maxParticles:
                randperm = np.random.choice(cardinality,
                                            size=(cardinality - self.params.cluster_maxParticles), replace=False)
                particles2redistribute = index_vector[0][randperm.astype(int)]
                tempSet = np.zeros([4, np.size(particles2redistribute)])

                for p in range(np.size(particles2redistribute)):
                    tempSet[:, p] = self.generate_particle()[:4]

                limited_set[:4, particles2redistribute] = tempSet
                self.IDX[particles2redistribute] = 0

    def calculate_filtered_state(self):
        """ Calculate filtered state, which contains positions of estimated updrafts

        """

        for i in range(1, self.cluster_num + 1):
            aCluster = self.particles[:, self.IDX == i]
            aCluster[4, :] = aCluster[4, :] / sum(aCluster[4, :])
            self.filtered_state[:, i - 1] = aCluster[:4, :] @ aCluster[4, :]
