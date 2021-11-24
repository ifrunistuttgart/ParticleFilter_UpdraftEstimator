## Particle filter-based multiple updraft estimator 

### Overview
This repository contains a python implementation of a particle filter-based
multiple updraft estimator, which was developed at the
 [Institute of Flight Mechanics and Controls (iFR)](https://www.ifr.uni-stuttgart.de/)
at the University of Stuttgart. It allows  estimating position, strength, and spread of several thermals at
once.

The estimator was used to localize real updrafts during flight-tests. It is capable of running on Raspberry Pi
Zero, which functions as a co-processor for the main Pixhawk 4 flight controller. The flight-tests were part
of the [Reinforcement Learning for Cross-Country Soaring](https://github.com/ifrunistuttgart/RL_CrossCountrySoaring)
project at the iFR. In this project, the estimated updrafts are fed into a hierarchical policy, which
autonomously covers the task of GPS-triangle soaring. 

More detailed information about the the hierarchical reinforcement learning approach to cross-country
soaring and the particle filter can be found in the following papers:

> Paper Hierarchical Reinforcement Learning

> Notter, S., Groß, P., Schrapel, P., and Fichter, W., “Multiple Thermal Updraft Estimation and Observability Analysis,” Journal
of Guidance, Control, and Dynamics, Vol. 43, No. 3, 2020, pp. 490–503. doi:10.2514/1.G004205

![Dummy image](resources/UpdraftEstimatorImage.PNG)


### Getting started
The actual implementation of the particle filter can be found in the *particle_filter*  folder. As it is not possible
to log the particles during the flight due to memory limitations, *run_postprocessing.py* calculates the
particle distribution from the flight data. The MATLAB script *showPostprocessingResult.m* visualizes the particles,
as well as the estimated updrafts during the flight test.
