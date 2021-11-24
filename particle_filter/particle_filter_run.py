'''
    Name: particle_filter_run

    Functionality: execute particle filter

    Description: establishes the communication to send and receive custom MAVLINK messages to/from Pixhawk and runs the particle filter updraft estimator
    
    Limitations: ---
         
    Project: Autonomous Soaring
    Author: Stefan Notter
    Date: 2021/05/10
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

#-------------------------- Hack to receive latest msg -------------------------------

def read_current_mavlink_msg(connection, msg_name):
    """ discards messages from stack to only return the latest message for further processing

    """

    msg_received = connection.recv_match(type=msg_name, blocking=True)
    
    while msg_received:
        msg_local_copy = msg_received
        msg_received = connection.recv_match(type=msg_name, blocking=False)
        
    return msg_local_copy

#--------------------------- Seeting Up PX Connection--------------------------------

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

#----------------------------------- PF Execution -----------------------------------

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
    filewriter.writerow(['Fail_Counter', 'Pi_Time', 'Pix_Time', 'Ctrl_Mode', 'AC_North', 'AC_East', 'AC_Down', 'Up_Local', 'Up_Detected', 'Up_1_North', 'Up_1_East', 'Up_1_Strength', 'Up_1_Spread', 'Up_2_North', 'Up_2_East', 'Up_2_Strength', 'Up_2_Spread', 'Up_3_North', 'Up_3_East', 'Up_3_Strength', 'Up_3_Spread', 'Up_4_North', 'Up_4_East', 'Up_4_Strength', 'Up_4_Spread', 'Up_5_North', 'Up_5_East', 'Up_5_Strength', 'Up_5_Spread', 'Up_6_North', 'Up_6_East', 'Up_6_Strength', 'Up_6_Spread'])
    
    
while True:

    t_start = perf_counter()
    
    try:
        
        # read vehicle local position message
        local_position_msg = read_current_mavlink_msg(connection=pixhawk, msg_name='LOCAL_POSITION_NED')
        vehicle_position = np.array([local_position_msg.x, local_position_msg.y, local_position_msg.z])
        
        # read local updraft estimate message
        local_updraft_msg = read_current_mavlink_msg(connection=pixhawk, msg_name='IFR_LOCAL_UPDRAFT_ESTIMATE')
        local_updraft_time = local_updraft_msg.time_usec
        local_updraft_estimate=local_updraft_msg.local_updraft
        # print("Local updraft msg received - time_usec: {}".format(local_updraft_time))
        
        # check whether controller mode has changed and possibly reset pf
        controller_mode_msg = read_current_mavlink_msg(connection=pixhawk, msg_name='CONTROLLER_STATUS_IFR')
        if controller_mode_msg.controllerMode != controller_mode:
            pixhawk.mav.ifr_particle_filter_updraft_estimates_send(local_updraft_time, 0, np.zeros([4, 1]), np.zeros([4, 1]) ,np.zeros([4, 1]), np.zeros([4, 1]),np.zeros([4, 1]), np.zeros([4, 1]))
            particle_filter.reset_filter()
            controller_mode = controller_mode_msg.controllerMode
            
            # wait until the glider has stabilized after mode change before calling the estimator, again
            t_reset = perf_counter() - t_start
            if t_passed < 10.0:
                sleep(10.0 - t_passed)
            
        # call particle filter
        pf_updraft_estimates, updraft_detected = particle_filter.run_filter_step(vehicle_position=vehicle_position, local_updraft_estimate=np.clip(local_updraft_estimate, a_min=0, a_max=None))  
        
        # write results to output message
        updraft_estimate_1 = np.array([pf_updraft_estimates[0, 0], pf_updraft_estimates[1, 0], pf_updraft_estimates[2, 0], pf_updraft_estimates[3, 0]])
        updraft_estimate_2 = np.array([pf_updraft_estimates[0, 1], pf_updraft_estimates[1, 1], pf_updraft_estimates[2, 1], pf_updraft_estimates[3, 1]])
        updraft_estimate_3 = np.array([pf_updraft_estimates[0, 2], pf_updraft_estimates[1, 2], pf_updraft_estimates[2, 2], pf_updraft_estimates[3, 2]])
        updraft_estimate_4 = np.array([pf_updraft_estimates[0, 3], pf_updraft_estimates[1, 3], pf_updraft_estimates[2, 3], pf_updraft_estimates[3, 3]])
        updraft_estimate_5 = np.array([pf_updraft_estimates[0, 4], pf_updraft_estimates[1, 4], pf_updraft_estimates[2, 4], pf_updraft_estimates[3, 4]])
        updraft_estimate_6 = np.array([pf_updraft_estimates[0, 5], pf_updraft_estimates[1, 5], pf_updraft_estimates[2, 5], pf_updraft_estimates[3, 5]])
        
        # send particle filter updraft estimates to pixhawk
        pixhawk.mav.ifr_particle_filter_updraft_estimates_send(local_updraft_time, updraft_detected, updraft_estimate_1, updraft_estimate_2 ,updraft_estimate_3, updraft_estimate_4,updraft_estimate_5, updraft_estimate_6)
        
        # append line to csv log file
        with open(log_file_name, 'a+') as log_file:
            filewriter = csv.writer(log_file, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
            filewriter.writerow([fail_counter, t_start, local_updraft_time, controller_mode, vehicle_position[0], vehicle_position[1], vehicle_position[2], local_updraft_estimate, updraft_detected,  updraft_estimate_1[0], updraft_estimate_1[1], updraft_estimate_1[2], updraft_estimate_1[3], updraft_estimate_2[0], updraft_estimate_2[1], updraft_estimate_2[2], updraft_estimate_2[3], updraft_estimate_3[0], updraft_estimate_3[1], updraft_estimate_3[2], updraft_estimate_3[3], updraft_estimate_4[0], updraft_estimate_4[1], updraft_estimate_4[2], updraft_estimate_4[3], updraft_estimate_5[0], updraft_estimate_5[1], updraft_estimate_5[2], updraft_estimate_5[3], updraft_estimate_6[0], updraft_estimate_6[1], updraft_estimate_6[2], updraft_estimate_6[3]])
            
    except:
    
        fail_counter += 1        
        continue
    
    # sleep for some time to meet desired update rate    
    t_passed = perf_counter() - t_start
    # print("Turnaround time: {:.5f} s.\n".format(t_passed))
    if t_passed < update_rate:
        sleep(update_rate - t_passed)
    
    