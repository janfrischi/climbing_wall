from Phidget22.Phidget import *
from Phidget22.Devices.VoltageRatioInput import *
import time
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import csv
from math import sqrt
import numpy as np
from scipy.integrate import cumulative_trapezoid
from scipy import signal
from scipy.signal import butter, filtfilt, argrelextrema
import serial
from scipy.signal import firwin, filtfilt
import socket
import select 

### This code has to be ran on the ESP32 board connected to the Phidget boards via USB.
### To do so use connect to Host and use ESP32 IP 192.168.1.201 -> Open a remote window
### bottom left corner of vscode and then run demonstration.py

#assuming 4 boards, whoch need to be calibrated indiviudally

# Gains Top Left
gain_x_TL = -757412564.29
gain_y_TL = 392070261.47
gain_z_TL = 2912917034.18

# Gains Top Right
gain_x_TR = -1946522934.4
gain_y_TR = 685862810.18
gain_z_TR = 641061587.03

# Gains Bottom Left
gain_x_BL = 2083499744.48*0.8
gain_y_BL = 351465288.30
gain_z_BL = -519850293.23

# Gains Bottom Right
gain_x_BR = 292070826.50
gain_y_BR = -338739044.93
gain_z_BR = -489063756.77


#if offsets not yet calculated!
def tare_scale_full_board(sensor00, sensor01, sensor02,sensor03, sensor04, sensor05, sensor06, sensor07, sensor08,sensor09, sensor10, sensor11,
                          measurements_x_TL, measurements_y_TL, measurements_z_TL,measurements_x_TR, 
                          measurements_y_TR, measurements_z_TR, measurements_x_BL, measurements_y_BL, 
                          measurements_z_BL, measurements_x_BR, measurements_y_BR, measurements_z_BR):
    print("Taring the scale. Please ensure no weight is applied.")
    time.sleep(2) #Allow time for the user to ensure no weight is applied
    # Take 20 measurements over 5 seconds
    for _ in range(20):
        measurements_x_TL.append(sensor00.getVoltageRatio())
        measurements_y_TL.append(sensor01.getVoltageRatio())
        measurements_z_TL.append(sensor02.getVoltageRatio())
        
        measurements_x_TR.append(sensor03.getVoltageRatio())
        measurements_y_TR.append(sensor04.getVoltageRatio())
        measurements_z_TR.append(sensor05.getVoltageRatio())

        measurements_x_BL.append(sensor06.getVoltageRatio())
        measurements_y_BL.append(sensor07.getVoltageRatio())
        measurements_z_BL.append(sensor08.getVoltageRatio())

        measurements_x_BR.append(sensor09.getVoltageRatio())
        measurements_y_BR.append(sensor10.getVoltageRatio())
        measurements_z_BR.append(sensor11.getVoltageRatio())
        
        time.sleep(0.25)  # Wait 0.25 seconds between measurements

def get_total_weight_full_board(sensor00, sensor01, sensor02, sensor03, sensor04, sensor05, sensor06, sensor07, sensor08,sensor09, sensor10, sensor11,
                                start_time, time_str, total_weight, offset_x_TL, offset_y_TL, offset_z_TL, offset_x_TR, offset_y_TR, offset_z_TR,
                                offset_x_BL, offset_y_BL, offset_z_BL, offset_x_BR, offset_y_BR, offset_z_BR, serial, inputCommand):
    inputCommand = ""
    try: 
        while True:          
            timestamp = time.time() - start_time
            #data_combined = [round(timestamp,2), round(total_weight,2), round(weight_x,0), round(weight_y,0), round(weight_z,0)]
        
            try:
                F_x_TL = round((sensor00.getVoltageRatio() - offset_x_TL) *  gain_x_TL) * 9.81 * 0.001 #convert to N
                F_y_TL = round((sensor01.getVoltageRatio() - offset_y_TL) *  gain_y_TL) * 9.81 * 0.001
                F_z_TL = round((sensor02.getVoltageRatio() - offset_z_TL) *  gain_z_TL) * 9.81 * 0.001
                
                F_x_TR = round((sensor03.getVoltageRatio() - offset_x_TR) *  gain_x_TR) * 9.81 * 0.001
                F_y_TR = round((sensor04.getVoltageRatio() - offset_y_TR) *  gain_y_TR) * 9.81 * 0.001
                F_z_TR = round((sensor05.getVoltageRatio() - offset_z_TR) *  gain_z_TR) * 9.81 * 0.001
                
                F_x_BL = round((sensor06.getVoltageRatio() - offset_x_BL) *  gain_x_BL) * 9.81 * 0.001
                F_y_BL = round((sensor07.getVoltageRatio() - offset_y_BL) *  gain_y_BL) * 9.81 * 0.001
                F_z_BL = round((sensor08.getVoltageRatio() - offset_z_BL) *  gain_z_BL) * 9.81 * 0.001
                
                F_x_BR = round((sensor09.getVoltageRatio() - offset_x_BR) *  gain_x_BR) * 9.81 * 0.001
                F_y_BR = round((sensor10.getVoltageRatio() - offset_y_BR) *  gain_y_BR) * 9.81 * 0.001
                F_z_BR = round((sensor11.getVoltageRatio() - offset_z_BR) *  gain_z_BR) * 9.81 * 0.001
                #Lets print all the forces to the website with a seperated ,
                data = f"{F_x_TL},{F_y_TL},{F_z_TL},{F_x_TR},{F_y_TR},{F_z_TR},{F_x_BL},{F_y_BL},{F_z_BL},{F_x_BR},{F_y_BR},{F_z_BR}\n"
                #print(f"[TX] {data.strip()}")
                serial.write(data.encode('utf-8'))
            except Exception as e:
                # Sensors may not have data available yet, skip this iteration
                time.sleep(0.01)
                continue
            '''
            with open(f'CSV\\combined_phidget_data_{time_str}.csv', 'a', newline = '') as file: #creates csv file in directory where code is run
                writer = csv.writer(file)
                writer.writerow(data_combined)
            '''    
            time.sleep(0.01)
        # add the follwoing exception: if(serial.available() > 0)  -> leave the function
            if (serial.in_waiting > 0):
                inputCommand = serial.readline().decode('utf-8').strip()
                print("Exiting force plotter")
                return inputCommand
    except KeyboardInterrupt:
        print("\nstopped by KeyboardInterrupt")

def get_hand_foot_forces(sensor00, sensor01, sensor02, sensor03, sensor04, sensor05, sensor06, sensor07, sensor08,sensor09, sensor10, sensor11,
                         offset_x_TL, offset_y_TL, offset_z_TL, offset_x_TR, offset_y_TR, offset_z_TR,
                         offset_x_BL, offset_y_BL, offset_z_BL, offset_x_BR, offset_y_BR, offset_z_BR,serial):
    hand_forces_list = []
    foot_forces_list = []
    #for self test during development
    time.sleep(3) #allow time for the user to prepare -> get on climbing wall
       
    try:
        print("Collecting hand and foot forces.") 
        while True:
            try:
                F_x_TL = round((sensor00.getVoltageRatio() - offset_x_TL) *  gain_x_TL) * 9.81 * 0.001 #convert to N
                F_y_TL = round((sensor01.getVoltageRatio() - offset_y_TL) *  gain_y_TL) * 9.81 * 0.001
                F_z_TL = round((sensor02.getVoltageRatio() - offset_z_TL) *  gain_z_TL) * 9.81 * 0.001
                
                F_x_TR = round((sensor03.getVoltageRatio() - offset_x_TR) *  gain_x_TR) * 9.81 * 0.001
                F_y_TR = round((sensor04.getVoltageRatio() - offset_y_TR) *  gain_y_TR) * 9.81 * 0.001
                F_z_TR = round((sensor05.getVoltageRatio() - offset_z_TR) *  gain_z_TR) * 9.81 * 0.001
                
                F_x_BL = round((sensor06.getVoltageRatio() - offset_x_BL) *  gain_x_BL) * 9.81 * 0.001
                F_y_BL = round((sensor07.getVoltageRatio() - offset_y_BL) *  gain_y_BL) * 9.81 * 0.001
                F_z_BL = round((sensor08.getVoltageRatio() - offset_z_BL) *  gain_z_BL) * 9.81 * 0.001
                
                F_x_BR = round((sensor09.getVoltageRatio() - offset_x_BR) *  gain_x_BR) * 9.81 * 0.001
                F_y_BR = round((sensor10.getVoltageRatio() - offset_y_BR) *  gain_y_BR) * 9.81 * 0.001
                F_z_BR = round((sensor11.getVoltageRatio() - offset_z_BR) *  gain_z_BR) * 9.81 * 0.001
                #Lets print all the forces to the website with a seperated ,
                data = f"{F_x_TL},{F_y_TL},{F_z_TL},{F_x_TR},{F_y_TR},{F_z_TR},{F_x_BL},{F_y_BL},{F_z_BL},{F_x_BR},{F_y_BR},{F_z_BR}\n"
                serial.write(data.encode('utf-8'))
                
                #foot_forces, hand_forces
                hand_forces_x = ((sensor00.getVoltageRatio() - offset_x_TL) * gain_x_TL + (sensor03.getVoltageRatio() - offset_x_TR) * gain_x_TR) *9.81*0.001
                hand_forces_y = ((sensor01.getVoltageRatio() - offset_y_TL) * gain_y_TL + (sensor04.getVoltageRatio() - offset_y_TR) * gain_y_TR) *9.81*0.001 
                hand_forces_z = ((sensor02.getVoltageRatio() - offset_z_TL) * gain_z_TL + (sensor05.getVoltageRatio() - offset_z_TR) * gain_z_TR) *9.81*0.001
                
                foot_forces_x = ((sensor06.getVoltageRatio() - offset_x_BL) * gain_x_BL +(sensor09.getVoltageRatio() - offset_x_BR) * gain_x_BR)*9.81 *0.001
                foot_forces_y = ((sensor07.getVoltageRatio() - offset_y_BL) * gain_y_BL +(sensor10.getVoltageRatio() - offset_y_BR) * gain_y_BR)*9.81 *0.001
                foot_forces_z = ((sensor08.getVoltageRatio() - offset_z_BL) * gain_z_BL +(sensor11.getVoltageRatio() - offset_z_BR) * gain_z_BR)*9.81 *0.001
                
                hand_forces_list.append([hand_forces_x,hand_forces_y,hand_forces_z])
                foot_forces_list.append([foot_forces_x,foot_forces_y,foot_forces_z])
            except Exception as e:
                # Sensors may not have data available yet, skip this iteration
                time.sleep(0.01)
                continue
                
            #adjust sampling rate if needed
            time.sleep(0.01)
        # add the follwoing exception: if(serial.available() > 0)  -> leave the function (jump is over)
            if (serial.in_waiting > 0):
                return hand_forces_list, foot_forces_list
    except KeyboardInterrupt:
        print("\nstopped by KeyboardInterrupt")

    return hand_forces_list, foot_forces_list

def calculate_jump_height_with_angle(foot_forces, hand_forces, mass, sampling_rate, wall_angle_degrees):
    """
    Calculate jump height from wall-oriented force data with angle correction.
    Uses consistent sign convention for forces and correct gravity handling.
    
    Parameters:
    foot_forces: numpy array of shape (n_samples, 3) with columns [Fx, Fy, Fz] in wall coordinates
                 These are REACTION forces (what sensors measure)
    hand_forces: numpy array of shape (n_samples, 3) with columns [Fx, Fy, Fz] in wall coordinates
                 These are REACTION forces (what sensors measure)
    mass: Climber's mass in kg
    sampling_rate: Sampling frequency in Hz
    wall_angle_degrees: Angle of wall from vertical (positive = overhanging)
    
    Returns:
    jump_height: Maximum jump height in meters (perpendicular to wall)
    time_series: Data for plotting
    """
    # Setup
    dt = 1.0 / sampling_rate
    time = np.arange(len(foot_forces)) / sampling_rate
    wall_angle = np.radians(wall_angle_degrees)
    
    # Transform forces from wall coordinates to global coordinates
    # Wall: +X is along wall (upward), +Z is perpendicular to wall (outward)
    # Global: +Z is vertical (DOWNWARD), +X is horizontal (away from wall)
    
    #Wall: +Y is perpendicular to wall (outward), +Z is vertical (DOWNWARD)
    #Global: +Z is vertical (DOWNWARD), +Y is horizontal (away from wall)
    
    # Foot forces transformation (for reaction forces)
    # For downward Z: multiply by -1 for the Z component to flip direction
    foot_global_z = foot_forces[:, 2] * np.cos(wall_angle) + foot_forces[:, 1] * np.sin(wall_angle)
    foot_global_x = foot_forces[:, 2] * np.sin(wall_angle) + foot_forces[:, 1] * np.cos(wall_angle)
    # Hand forces transformation (for reaction forces)
    hand_global_z = hand_forces[:, 2] * np.cos(wall_angle) + hand_forces[:, 1] * np.sin(wall_angle)
    hand_global_x = hand_forces[:, 2] * np.sin(wall_angle) + hand_forces[:, 1] * np.cos(wall_angle)
    
    # Total vertical and horizontal forces
    total_force_z = foot_global_z + hand_global_z
    total_force_x = foot_global_x + hand_global_x
    
    # Calculate net force and acceleration
    # In downward Z system, weight is positive in Z direction
    still_end = int(1 * sampling_rate)  # First 1 second
    mean_weight_force = np.mean(total_force_z[:still_end*3])
    weight = mass * 9.81  # Positive because gravity acts in +Z direction
    net_force_z = total_force_z - mean_weight_force  # Subtract weight force (downward)
    accel_z = net_force_z / mass          # Divide by mass
    
    # Baseline correction
    still_end = int(1 * sampling_rate)  # First 1 second
    accel_baseline = np.mean(accel_z[:still_end])
    accel_z = accel_z - accel_baseline# Remove baseline
    

    i_max_accel = np.argmax(foot_global_z) # Find index of maximum acceleration after 1 second
    #find maximum occuring after 4 seconds
    #for testing alone -> need time to climb to the wall
    #i_max_accel = np.argmax(foot_global_z)   # Offset by still_end to get correct index
    # Define window for slope calculation
    window_start = max(0, i_max_accel - 200)

    # Initialize slope array
    slope = np.zeros(len(accel_z))
    for i in range(window_start, i_max_accel):
        if i + 1 < len(accel_z):
            slope[i] = (foot_global_z[i] - foot_global_z[i - 1])  # 100 Hz → dt = 0.01s

    slope_window = slope[window_start:i_max_accel]
    # check where slope is bigger than 10 for the first time
    for i in range(len(slope_window)):
        if slope_window[i] > 10:
            i_max_change_local = i
            break
     # Map back to global index (within full time or accel_z arrays)
    i_max_change = window_start + i_max_change_local  # +1 for diff offset
    # print results for debugging
    print(f"Index of first significant slope change: {i_max_change}, Time: {time[i_max_change]:.3f} s")
    print(f"Value of slope at this index: {slope[i_max_change]:.3f} m/s²")
    print(f"foot force at this index: {foot_global_z[i_max_change]:.3f} N")
    print(f"max acceleration index: {i_max_accel}, Time: {time[i_max_accel]:.3f} s")
    print(f"Value of acceleration at this index: {accel_z[i_max_accel]:.3f} m/s²")
    print(f"foot force at this index: {foot_global_z[i_max_accel]:.3f} N")
    accel_z[:i_max_change] = 0  # Set all values before true_start to 0
    # First integration: acceleration → velocity
    velocity_z = cumulative_trapezoid(accel_z, dx=dt, initial=0)
    
    # Velocity baseline correction
    velocity_baseline = np.mean(velocity_z[:still_end])
    velocity_z = velocity_z - velocity_baseline  # Remove baseline
    
    # Second integration: velocity → position
    position_z = cumulative_trapezoid(velocity_z, dx=dt, initial=0)
    
    # Position baseline correction
    position_baseline = np.mean(position_z[:still_end])
    position_z = position_z - position_baseline
    
    #detect the index at which the acceceleration crosses zero for the first time after the maximum acceleration
    
    zero_crossings = np.where(np.diff(np.sign(total_force_z)))[0]
    
    #minimum index different zero crossing - i_max_accel
    # only get zero crossings after i_max_accel
    
    if len(zero_crossings[zero_crossings>i_max_accel]) == 0:
        print("No zero crossings found after maximum acceleration.")
        # check where total_force_z falls below a certain threshold after i_max_accel
        threshold = 20  # Define a threshold for zero crossing
        forces_z = total_force_z[i_max_accel:]  # Forces after maximum acceleration
        zero_crossings = np.where(forces_z < threshold)[0]
        zero_crossing_index = zero_crossings[0] + i_max_accel  # Adjust index to global scale
        true_takeoff_index = zero_crossing_index
        return true_takeoff_index
    else:
        zero_crossing_index = zero_crossings[zero_crossings > i_max_accel][0]
        # Find the index of the first zero crossing after i_max_accel
        true_takeoff_index = zero_crossing_index
       
    
    print(f"True takeoff index: {true_takeoff_index}, Velocity at takeoff: {velocity_z[true_takeoff_index]:.3f} m/s")
    #velcoity based calculated height
    max_velocity = velocity_z[true_takeoff_index]
    jump_height_velocity = (max_velocity ** 2) / (2 * 9.81)  # Using v^2 = 2gh
    print(f"Jump height based on velocity: {jump_height_velocity:.3f} m")
    # Calculate integrated position-based jump height (now our primary result)
    jump_height = 0.0
    if true_takeoff_index is not None:
        peak_height = np.max(position_z) - position_z[true_takeoff_index]  # Position at true takeoff index
        # Apply wall angle correction to get wall-perpendicular height
        jump_height = peak_height * np.cos(wall_angle)
    
    return jump_height

def fir_lowpass_filter(data, cutoff, fs, width=5, ripple_db=60):
    nyq = 0.5 * fs
    numtaps, beta = signal.kaiserord(ripple_db, width / nyq)
    fir_coeff = firwin(numtaps, cutoff / nyq, window=('kaiser', beta))
    return filtfilt(fir_coeff, [1.0], data)

        
def main():
    #Input Angle of Inclination
    sampling_rate = 100 #Hz
    calibrationMode = False
    
    # Template for campusboard
    # top-left (TL) until bottom-right (BR)
    # define sensors
    ser = serial.Serial('/dev/ttyGS0', 115200, timeout=1)
    
    sensor00 = VoltageRatioInput() #x-axis TL
    sensor01 = VoltageRatioInput() #y-axis TL
    sensor02 = VoltageRatioInput() #z-axis TL

    sensor03 = VoltageRatioInput() #x-axis TR
    sensor04 = VoltageRatioInput() #y-axis TR
    sensor05 = VoltageRatioInput() #z-axis TR

    sensor06 = VoltageRatioInput() #x-axis BL
    sensor07 = VoltageRatioInput() #y-axis BL
    sensor08 = VoltageRatioInput() #z-axis BL

    sensor09 = VoltageRatioInput() #x-axis BR
    sensor10 = VoltageRatioInput() #y-axis BR
    sensor11 = VoltageRatioInput() #z-axis BR

    #assign serial number of phidget
    sensor00.setDeviceSerialNumber(481661) #TO-Do assign serial number for TL-board
    sensor01.setDeviceSerialNumber(481661) #TO-Do assign serial number for TL-board
    sensor02.setDeviceSerialNumber(481661) #TO-Do assign serial number for TL-board

    sensor03.setDeviceSerialNumber(293698) #TO-Do assign serial number for TR-board
    sensor04.setDeviceSerialNumber(293698) #TO-Do assign serial number for TR-board
    sensor05.setDeviceSerialNumber(293698) #TO-Do assign serial number for TR-board

    sensor06.setDeviceSerialNumber(588832) #TO-Do assign serial number for BL-board
    sensor07.setDeviceSerialNumber(588832) #TO-Do assign serial number for BL-board
    sensor08.setDeviceSerialNumber(588832) #TO-Do assign serial number for BL-board

    sensor09.setDeviceSerialNumber(293701) #TO-Do assign serial number for BR-board
    sensor10.setDeviceSerialNumber(293701) #TO-Do assign serial number for BR-board
    sensor11.setDeviceSerialNumber(293701) #TO-Do assign serial number for BR-board
    
    
    #assign channel number for each sensor
    #TL
    sensor00.setChannel(0) # channel 0 for x-axis
    sensor01.setChannel(1) # channel 1 for y-axis
    sensor02.setChannel(2) # channel 2 for z-axis
    
    #TR
    sensor03.setChannel(0) # channel 0 for x-axis
    sensor04.setChannel(1) # channel 1 for y-axis
    sensor05.setChannel(2) # channel 2 for z-axis
    #BL
    sensor06.setChannel(0) # channel 0 for x-axis
    sensor07.setChannel(1) # channel 1 for y-axis
    sensor08.setChannel(2) # channel 2 for z-axis
    #BR
    sensor09.setChannel(0) # channel 0 for x-axis
    sensor10.setChannel(1) # channel 1 for y-axis
    sensor11.setChannel(2) # channel 2 for z-axis
    
    print("Opening Sensors - Wait")
    # Open sensors with delays to avoid USB/threading conflicts
    try:
        sensor00.openWaitForAttachment(15000)
        time.sleep(0.1)
        sensor01.openWaitForAttachment(15000)
        time.sleep(0.1)
        sensor02.openWaitForAttachment(15000)
        time.sleep(0.1)
        print("TL board opened successfully")
        
        sensor03.openWaitForAttachment(15000)
        time.sleep(0.1)
        sensor04.openWaitForAttachment(15000)
        time.sleep(0.1)
        sensor05.openWaitForAttachment(15000)
        time.sleep(0.1)
        print("TR board opened successfully")
        
        sensor06.openWaitForAttachment(15000)
        time.sleep(0.1)
        sensor07.openWaitForAttachment(15000)
        time.sleep(0.1)
        sensor08.openWaitForAttachment(15000)
        time.sleep(0.1)
        print("BL board opened successfully")
        
        sensor09.openWaitForAttachment(15000)
        time.sleep(0.1)
        sensor10.openWaitForAttachment(15000)
        time.sleep(0.1)
        sensor11.openWaitForAttachment(15000)
        print("BR board opened successfully")
        print("All sensors opened successfully!")
    except Exception as e:
        print(f"Error opening sensors: {e}")
        print("Cleaning up and exiting...")
        # Close any sensors that were opened
        for sensor in [sensor00, sensor01, sensor02, sensor03, sensor04, sensor05, 
                       sensor06, sensor07, sensor08, sensor09, sensor10, sensor11]:
            try:
                if sensor.getAttached():
                    sensor.close()
            except:
                pass
        raise
    
    print("Set data interval")
    # Wait a moment for all sensors to stabilize
    time.sleep(0.5)
    
    # Set the data interval for the application with verification
    sensors = [sensor00, sensor01, sensor02, sensor03, sensor04, sensor05, 
               sensor06, sensor07, sensor08, sensor09, sensor10, sensor11]
    
    for i, sensor in enumerate(sensors):
        try:
            if sensor.getAttached():
                sensor.setDataInterval(10)
            else:
                print(f"Warning: Sensor {i} not attached, skipping data interval setting")
        except Exception as e:
            print(f"Error setting data interval for sensor {i}: {e}")
            raise
    
    print("Data intervals set successfully")
    
    # Wait for sensors to stabilize and start providing valid data
    print("Waiting for sensors to stabilize...")
    time.sleep(2)
    
    # Verify all sensors are providing data
    max_retries = 10
    for retry in range(max_retries):
        try:
            # Try reading from all sensors
            for i, sensor in enumerate(sensors):
                if sensor.getAttached():
                    _ = sensor.getVoltageRatio()
            print("All sensors ready!")
            break
        except Exception as e:
            if retry < max_retries - 1:
                print(f"Sensors not ready yet, waiting... (attempt {retry+1}/{max_retries})")
                time.sleep(1)
            else:
                print(f"Error: Sensors failed to stabilize after {max_retries} attempts")
                raise
    

    start_time = time.time()
    time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Initialize lists to store measurements
    measurements_x_TL = []
    measurements_y_TL = []
    measurements_z_TL = []

    measurements_x_TR = []
    measurements_y_TR = []
    measurements_z_TR = []
    
    measurements_x_BL = []
    measurements_y_BL = []
    measurements_z_BL = []

    measurements_x_BR = []
    measurements_y_BR = []
    measurements_z_BR = []
    
    total_weight = 0 #this is the calculated mass from measurements
    
    # Initialize mean offsets
    offset_x_TL = 0
    offset_y_TL = 0
    offset_z_TL = 0
    offset_x_TR = 0
    offset_y_TR = 0
    offset_z_TR = 0
    offset_x_BL = 0
    offset_y_BL = 0
    offset_z_BL = 0
    offset_x_BR = 0
    offset_y_BR = 0
    offset_z_BR = 0
    
    #add a while true loop that runs until the user presses a button -> program runs all the time if no command just send forces to website, otherwise follow command
    #adjust this with real name! 
    inputCommand = ""
    mass = 0.0
    angle_deg = 0.0
    try:
         while True:
            if ser.in_waiting > 0 or inputCommand != "":
                incomingByte = ser.read().decode('utf-8')

                if incomingByte == '\n' or inputCommand != "":
                    
                    if inputCommand == "":
                        inputCommand = ser.readline().decode('utf-8').strip()  # Get next input (mass or angle)
                        # Handle mass input
                    if inputCommand.startswith("mass:"):
                            try:
                                value_str = inputCommand.split("mass:")[1]
                                mass = int(value_str)
                                print(f"Mass value updated to: {mass}")
                            except ValueError:
                                print("Invalid mass value received.")
                            # Handle angle input
                            inputCommand = ""
                    elif inputCommand.startswith("angle:"):
                            try:
                                    value_str = inputCommand.split("angle:")[1]
                                    angle_deg = int(value_str)
                                    print(f"Angle value updated to: {angle_deg}")
                            except ValueError:
                                    print("Invalid angle value received.")
                            inputCommand = ""
                    elif inputCommand == "start_jump":
                            
                            # Start measuring data
                            foot_forces_list = []
                            hand_forces_list = []
                            # Call your force-reading function
                            print("collecting data for jump")
                            hand_forces_list, foot_forces_list = get_hand_foot_forces(sensor00, sensor01, sensor02,sensor03, sensor04, sensor05, sensor06, sensor07, sensor08,sensor09, sensor10, sensor11,
                                    offset_x_TL, offset_y_TL, offset_z_TL, offset_x_TR, offset_y_TR, offset_z_TR,
                                    offset_x_BL, offset_y_BL, offset_z_BL, offset_x_BR, offset_y_BR, offset_z_BR,ser) 
                            foot_forces = np.array(foot_forces_list)
                            hand_forces = np.array(hand_forces_list)
                            #lets filter the forces with a low pass filter -> smooth the data
                            
                            
                            
                            print("going to calculate jump")      
                            jump_height = calculate_jump_height_with_angle(foot_forces, hand_forces, mass, sampling_rate, angle_deg)
                                #Print Height to Website
                            #jump_height = 0.05
                            jump =round(jump_height * 100)    
                            ser.write(f"jump: {jump}\n".encode('utf-8'))
                            
                            
                        #add calibration input
                    elif (inputCommand == "calibrate"):
                            calibrationMode = True
                            #To-Do: add calibration function
                            tare_scale_full_board(sensor00, sensor01, sensor02,sensor03, sensor04, sensor05, sensor06, sensor07, sensor08,sensor09, sensor10, sensor11,
                                    measurements_x_TL, measurements_y_TL, measurements_z_TL,measurements_x_TR, 
                                    measurements_y_TR, measurements_z_TR, measurements_x_BL, measurements_y_BL, 
                                    measurements_z_BL, measurements_x_BR, measurements_y_BR, measurements_z_BR)
                            calibrationMode = False
                            
                            offset_x_TL = sum(measurements_x_TL) / len(measurements_x_TL)
                            offset_y_TL = sum(measurements_y_TL) / len(measurements_y_TL)
                            offset_z_TL = sum(measurements_z_TL) / len(measurements_z_TL)
                            offset_x_TR = sum(measurements_x_TR) / len(measurements_x_TR)
                            offset_y_TR = sum(measurements_y_TR) / len(measurements_y_TR)
                            offset_z_TR = sum(measurements_z_TR) / len(measurements_z_TR)
                            offset_x_BL = sum(measurements_x_BL) / len(measurements_x_BL)
                            offset_y_BL = sum(measurements_y_BL) / len(measurements_y_BL)
                            offset_z_BL = sum(measurements_z_BL) / len(measurements_z_BL)
                            offset_x_BR = sum(measurements_x_BR) / len(measurements_x_BR)
                            offset_y_BR = sum(measurements_y_BR) / len(measurements_y_BR)
                        
                            offset_z_BR = sum(measurements_z_BR) / len(measurements_z_BR)
                    inputCommand = "" #Clear command buffer
                else:
                    inputCommand += incomingByte
            else:
                #To-Do: Publish forces inside the function and also leave the function as soon as there is a command in the buffer
                print("Going into force calculator/send function")
                inputCommand = get_total_weight_full_board(sensor00, sensor01, sensor02, sensor03, sensor04, sensor05, sensor06, sensor07, sensor08,sensor09, sensor10, sensor11,
                                            start_time, time_str, total_weight, offset_x_TL, offset_y_TL, offset_z_TL, offset_x_TR, offset_y_TR, offset_z_TR,
                                            offset_x_BL, offset_y_BL, offset_z_BL, offset_x_BR, offset_y_BR, offset_z_BR, ser, inputCommand)
                print(inputCommand)
    except KeyboardInterrupt:
        print("\nstopped by KeyboardInterrupt")
    finally:
        # Safely close all sensors
        print("Closing sensors...")
        for sensor in [sensor00, sensor01, sensor02, sensor03, sensor04, sensor05, 
                       sensor06, sensor07, sensor08, sensor09, sensor10, sensor11]:
            try:
                if sensor.getAttached():
                    sensor.close()
            except Exception as e:
                print(f"Error closing sensor: {e}")
        print("All sensors closed.")
    

main()