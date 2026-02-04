from Phidget22.Phidget import *
from Phidget22.Devices.VoltageRatioInput import *
import time
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import csv
from math import sqrt
import numpy as np


#Insert your gain value from the Phidget Control Panel

#Add gain values here after calibration
gain_x =  298396265.7956946 #value for x direction
gain_y = -290281706.4735825#value for y direction
gain_z = 288754731.77752054#value for z direction


def tare_scale(sensor00, sensor01, sensor02,  measurements_x, measurements_y, measurements_z, time_str):
    """
    Tares the scale by calculating the initial offset for x, y, and z directions.
    Returns the offsets for each direction.
    """
    print("Taring the scale. Please ensure no weight is applied.")
    time.sleep(2)  # Allow time for the user to ensure no weight is applied

    # Take 20 measurements over 5 seconds
    for _ in range(20):
        voltage_x = sensor00.getVoltageRatio()
        voltage_y = sensor01.getVoltageRatio()
        voltage_z = sensor02.getVoltageRatio()

        measurements_x.append(voltage_x)
        measurements_y.append(voltage_y)
        measurements_z.append(voltage_z)

        time.sleep(0.25)  # Wait 0.25 seconds between measurements

def get_total_weight(sensor00, sensor01, sensor02, start_time, time_str, total_weight, offset_x, offset_y, offset_z, alpha_rad):
    print("offset_y", offset_y)
    try: 
        while True:
            weight_x = (sensor00.getVoltageRatio() - offset_x) * gain_x
            weight_y = (sensor01.getVoltageRatio() - offset_y) * gain_y 
            weight_z = (sensor02.getVoltageRatio() - offset_z) * gain_z
            total_weight = sqrt(weight_x**2 + weight_y**2 + weight_z**2)
            #data_combined = [round(timestamp,2), round(total_weight,2), round(weight_x,0), round(weight_y,0), round(weight_z,0)]
            print('measured_weight: ', total_weight, "x:", weight_x, "y:", weight_y, "z:", weight_z)
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nstopped by KeyboardInterrupt")

def main():
    sensor00 = VoltageRatioInput() #x-axis
    sensor01 = VoltageRatioInput() #x-axis	
    sensor02 = VoltageRatioInput() #y-axis
   
    # Configuratiom Top Left
    
    sensor00.setDeviceSerialNumber(481661) #replace with your actual serial number
    sensor01.setDeviceSerialNumber(481661) #replace with your actual serial number	
    sensor02.setDeviceSerialNumber(481661) #replace with your actual serial number
   
    sensor00.setChannel(0) #x-axis -> channel 0 at the Phidget board
    sensor01.setChannel(1) #y-axis -> channel 1 at the Phidget board
    sensor02.setChannel(2) #z-axis -> channel 2 at the Phidget board 

    #Configuration Top Right
    
    """ sensor00.setDeviceSerialNumber(293698) #replace with your actual serial number
    sensor01.setDeviceSerialNumber(293698) #replace with your actual serial number
    sensor02.setDeviceSerialNumber(293698) #replace with your actual serial number
    sensor00.setChannel(0) #x-axis -> channel 3 at the Phidget board
    sensor01.setChannel(1) #y-axis -> channel 1 at the Phidget board
    sensor02.setChannel(2) #z-axis -> channel 2 at the Phidget board  """
    
    #Configuration Bottom Left

    """ sensor00.setDeviceSerialNumber(588832) #replace with your actual serial number
    sensor01.setDeviceSerialNumber(588832) #replace with your actual serial number
    sensor02.setDeviceSerialNumber(588832) #replace with your actual serial number
    sensor00.setChannel(0) #x-axis -> channel 0 at the Phidget board
    sensor01.setChannel(1) #y-axis -> channel 1 at the Phidget
    sensor02.setChannel(2) #z-axis -> channel 2 at the Phidget board  """
    
    #Configuration Bottom Right
    
    """ sensor00.setDeviceSerialNumber(293701) #replace with your actual serial number
    sensor01.setDeviceSerialNumber(293701) #replace with your actual serial number
    sensor02.setDeviceSerialNumber(293701) #replace with your actual serial number
    sensor00.setChannel(0) #x-axis -> channel 0 at the Phidget board
    sensor01.setChannel(1) #y-axis -> channel 1 at the Phidget
    sensor02.setChannel(2) #z-axis -> channel 2 at the Phidget board  """
    
    
    
    #open the devices
    sensor00.openWaitForAttachment(5000)
    sensor01.openWaitForAttachment(5000)
    sensor02.openWaitForAttachment(5000)
    


    #Set the data interval for the application
    sensor00.setDataInterval(1000)
    sensor01.setDataInterval(1000)
    sensor02.setDataInterval(1000)
    

    start_time = time.time()
    time_str = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Initialize lists to store measurements
    measurements_x = []
    measurements_y = []
    measurements_z = []

    tare_scale(sensor00, sensor01, sensor02, measurements_x, measurements_y, measurements_z, time_str)
    print("Taring Complete")

    # Calculate mean offsets
    offset_x = sum(measurements_x) / len(measurements_x)
    offset_y = sum(measurements_y) / len(measurements_y)
    offset_z = sum(measurements_z) / len(measurements_z)

    print(f"Offsets - X: {offset_x}, Y: {offset_y}, Z: {offset_z}")
    # Save offsets to a CSV file with a timestamp in the filename
    filename = f"tare_offsets_{time_str}.csv"
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Direction", "Offset"])
        writer.writerow(["X", offset_x])
        writer.writerow(["Y", offset_y])
        writer.writerow(["Z", offset_z])


    total_weight = 0
    print("Lets start the validation process")
    i = input("What is the inclination angle? (0, 15, 30, 45) ")
    alpha_rad = float(i) * (3.14159 / 180)  # Convert degrees to radians
    get_total_weight(sensor00, sensor01, sensor02, start_time, time_str, total_weight, offset_x, offset_y, offset_z, alpha_rad)

    try:
        input("Press Enter to Stop\n")
    except (Exception, KeyboardInterrupt):
        pass

    sensor00.close()
    sensor01.close()
    sensor02.close()

main()