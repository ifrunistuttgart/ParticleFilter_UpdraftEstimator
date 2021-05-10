'''
    Name: particle_filter_run

    Functionality: execute particle filter

    Description: establishes the communication to send and receive custom MAVLINK messages to/from Pixhawk and runs the particle filter updraft mestimator
    
    Limitations: ---
         
    Project: Autonomous Soaring
    Author: Stefan Notter
    Date: 2021/05/04
'''

from time import sleep
from time import perf_counter
from time import time

from pymavlink import mavutil
import os       # for setting environment variable
import csv      # logging data as csv file
import datetime # name log file according to time

import numpy as np
import scipy.io as sio
import particle_filter

#---------------------------Seeting Up PX Connection--------------------------------

print('------------------------------------------------------------\n')
print('Starting Communication... \n')

#Waiting time for exceptions
exception_time = 1.0

px4_flag = True

while px4_flag:
    try:
        print('Connection Settings for PX4 are being established')

        os.environ["MAVLINK_DIALECT"] = "common"
        os.environ["MAVLINK20"] = "1"

        mavutil.set_dialect("common")

        serialPort = "/dev/serial0"
        baudRate = 57600
            
        # connect to pixhawk at the specified serial port
        print('Connecting to Pixhawk...\n')
        pixhawk = mavutil.mavlink_connection(serialPort, baudRate)
        print('Connected:',pixhawk)
            
        # wait for the heartbeat msg to find the system ID
        print('Waiting for Heartbeat...\n')
        pixhawk.wait_heartbeat()
        px4_flag = False
        print('Connected to PX4: Heartbeat received! \n')
           
    except:
        sleep(exception_time)
        continue
        
# msg = None
# while not msg:
    # pixhawk.mav.ping_send(
        # int(time() * 1e6), # Unix time in microseconds
        # 0, # Ping number
        # 0, # Request ping of all systems
        # 0 # Request ping of all components
    # )
    # msg = pixhawk.recv_match()
    # sleep(0.5)
# print('Ping send.\n')

#--------------------------- PF Execution Test -------------------------------- 

# create filter instance
particle_filter = particle_filter.ParticleFilter()
update_rate = particle_filter.params.tau

# set further variables
controller_mode = 0
fail_counter = 0
step = 0

# create csv file for data logging
if not os.path.exists('log_files'):
    os.mkdir('log_files')
    
log_file_name = os.path.join('log_files', 'log_' + datetime.datetime.now().strftime('%d-%B-%Y_%H-%M') + '.csv')
with open(log_file_name, 'w+') as log_file:
    filewriter = csv.writer(log_file, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
    filewriter.writerow(['Raspi Time', 'Fail Counter', 'Corresponding Mav. Msg. Time', 'Controller Mode', 'Vehicle Local NED-Position', 'Local Updraft Estimate', 'Updraft Detected', 'PF Updraft Estimates'])
    

    
# for step in range(0, 1000):
while True:

    t_start = perf_counter()
    step += 1
    

    try:
        
        # read messages relevant for particle filter updraft estimation
        local_position_msg = pixhawk.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        # local_position_msg = pixhawk.messages['LOCAL_POSITION_NED']
        print("Local position msg received - north: {:.5f} \t east: {:.5f}".format(local_position_msg.x, local_position_msg.y))
        vehicle_position = np.array([local_position_msg.x, local_position_msg.y, local_position_msg.z])
        
        # Check wether controller mode has changed and reset pf
        controller_mode_msg = pixhawk.recv_match(type='CONTROLLER_STATUS_IFR', blocking=True)
        # controller_mode_msg = pixhawk.messages['CONTROLLER_STATUS_IFR']
        print("Controller mode msg received - mode: {}".format(controller_mode_msg.controllerMode))
        if controller_mode_msg.controllerMode != controller_mode:
            particle_filter.reset_filter()
            controller_mode = controller_mode_msg.controllerMode
        
        local_updraft_msg = pixhawk.recv_match(type='IFR_LOCAL_UPDRAFT_ESTIMATE', blocking=True)
        # local_updraft_time = pixhawk.messages['IFR_LOCAL_UPDRAFT_ESTIMATE'].time_usec
        local_updraft_time = local_updraft_msg.time_usec
        print("Local updraft msg received - time_usec: {}".format(local_updraft_time))
        timestamp=pixhawk.time_since('IFR_LOCAL_UPDRAFT_ESTIMATE')
        print("Time since: {}".format(timestamp))
        # local_updraft_updraft = pixhawk.messages['IFR_LOCAL_UPDRAFT_ESTIMATE'].local_updraft
        # print("Local updraft msg received - updraft: {}\n".format(local_updraft_updraft))
        
        # local_updraft_time = pixhawk.messages['IFR_LOCAL_UPDRAFT_ESTIMATE'].time_usec
        # print("Local updraft msg received - time_usec: {}".format(local_updraft_time))
        # timestamp=pixhawk.time_since('IFR_LOCAL_UPDRAFT_ESTIMATE')
        # print("Time since: {}".format(timestamp))
        # local_updraft_updraft = pixhawk.messages['IFR_LOCAL_UPDRAFT_ESTIMATE'].local_updraft
        # print("Local updraft msg received - updraft: {}\n\n".format(local_updraft_updraft))

        local_updraft_estimate=local_updraft_msg.local_updraft
        
        # call particle filter
        pf_updraft_estimates, updraft_detected = particle_filter.run_filter_step(vehicle_position=vehicle_position, local_updraft_estimate=local_updraft_estimate)  
        print("Updraft detected: {}.".format(updraft_detected))
        
        # write results to output message
        time_usec = local_updraft_time # synchronize with local updraft estimate msg - does that make sense?
        # updraft_detected = 0
        # updraft_estimate_1 = np.array([step, 0., 0., 0.])
        # updraft_estimate_2 = np.array([0., 0., 0., 0.])
        # updraft_estimate_3 = np.array([0., 0., 0., 0.])
        # updraft_estimate_4 = np.array([0., 0., 0., 0.])
        # updraft_estimate_5 = np.array([0., 0., 0., 0.])
        # updraft_estimate_6 = np.array([0., 0., 0., 0.])
        updraft_estimate_1 = np.array([pf_updraft_estimates[0, 0], pf_updraft_estimates[1, 0], pf_updraft_estimates[2, 0], pf_updraft_estimates[3, 0]])
        updraft_estimate_2 = np.array([pf_updraft_estimates[0, 1], pf_updraft_estimates[1, 1], pf_updraft_estimates[2, 1], pf_updraft_estimates[3, 1]])
        updraft_estimate_3 = np.array([pf_updraft_estimates[0, 2], pf_updraft_estimates[1, 2], pf_updraft_estimates[2, 2], pf_updraft_estimates[3, 2]])
        updraft_estimate_4 = np.array([pf_updraft_estimates[0, 3], pf_updraft_estimates[1, 3], pf_updraft_estimates[2, 3], pf_updraft_estimates[3, 3]])
        updraft_estimate_5 = np.array([pf_updraft_estimates[0, 4], pf_updraft_estimates[1, 4], pf_updraft_estimates[2, 4], pf_updraft_estimates[3, 4]])
        updraft_estimate_6 = np.array([pf_updraft_estimates[0, 5], pf_updraft_estimates[1, 5], pf_updraft_estimates[2, 5], pf_updraft_estimates[3, 5]])
        
        # send particle filter updraft estimates to pixhawk
        # pixhawk.mav.ifr_particle_filter_updraft_estimates_send(time_usec, updraft_detected, updraft_estimate_1, updraft_estimate_2 ,updraft_estimate_3, updraft_estimate_4,updraft_estimate_5, updraft_estimate_6)
        pixhawk.mav.ifr_particle_filter_updraft_estimates_send(time_usec, updraft_detected, updraft_estimate_1, updraft_estimate_2 ,updraft_estimate_3, updraft_estimate_4,updraft_estimate_5, updraft_estimate_6)
        print("Particle filter output send.")
        
        # append line to csv log file
        with open(log_file_name, 'a+') as log_file:
            filewriter = csv.writer(log_file, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
            filewriter.writerow([t_start, fail_counter, time_usec, controller_mode, vehicle_position, local_updraft_estimate, updraft_detected, updraft_estimate_1, updraft_estimate_2 ,updraft_estimate_3, updraft_estimate_4,updraft_estimate_5, updraft_estimate_6])

        #Sleep for some time
        t_passed = perf_counter() - t_start
        print("Turnaround time: {:.5f} s.\n".format(t_passed))
        if t_passed < update_rate:
            sleep(update_rate - t_passed)
        
    except:
    
        fail_counter += 1
        
        t_passed = perf_counter() - t_start
        if t_passed < update_rate:
            sleep(update_rate - t_passed)
        
        continue
        
print("Failed attemps to receive or send a message: {}".format(int(fail_counter)))