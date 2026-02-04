from Phidget22.Phidget import *
from Phidget22.Devices.VoltageRatioInput import *
import time
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import csv
from math import sqrt
import matplotlib.pyplot as plt
import numpy as np



def get_voltage(sensor00, sensor01, sensor02, voltages, mean_voltages, i):
    voltages_x = []
    voltages_y = []
    voltages_z = []

    for n in range(20):
        voltage_x = sensor00.getVoltageRatio()
        voltage_y = sensor01.getVoltageRatio()
        voltage_z = sensor02.getVoltageRatio()

        if i == "x":
            voltages_x.append(voltage_x)
        elif i == "y":
            voltages_y.append(voltage_y)
        elif i == "z": 
            voltages_z.append(voltage_z)

        if n == 10: #check during calibration if other directions are close to 0
            print("voltage_x", voltage_x) 
            print("voltage_y", voltage_y)
            print("voltage_z", voltage_z)
        
        time.sleep(0.25)

    if i == "x":
        voltages = voltages_x
    elif i == "y":
        voltages = voltages_y
    elif i == "z":
        voltages = voltages_z

    mean_voltages.append(sum(voltages) / len(voltages))
    print(sum(voltages) / len(voltages))

def plot_calibration_curve(mean_voltages, weights, gain, offset):
    x = np.linspace(min(mean_voltages), max(mean_voltages), 100)  # Generate 100 points between min and max voltage
    y = gain * x + offset  # Calculate the corresponding weights

    plt.figure(figsize=(8, 6))
    plt.plot(mean_voltages, weights, 'o', label='Calibration Data', color='b')  # Original data points
    plt.plot(x, y, '-', label=f'Fitted Line: y = {gain:.2f}x + {offset:.2f}', color='r')  # Fitted line

    # Add labels and title
    plt.xlabel('Mean Voltage (V)')
    plt.ylabel('Weight (g)')
    plt.title('Calibration Curve: Mean Voltage vs. Weight')
    plt.grid(True)
    plt.legend()

    # Show the plot
    plt.show()

# Save calibration data to a CSV file->adjust the filename as needed (TL, TR, BL, BR)
def save_calibration_data(mean_voltages, gain, offset, filename="calibration_data_board_TL.csv"):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        
        # Write headers
        writer.writerow(["Mean Voltages (V)", "Gain", "Offset"])
        
        # Write mean voltages
        for voltage in mean_voltages:
            writer.writerow([voltage])
        
        # Write gain and offset
        writer.writerow([])
        writer.writerow(["Gain", gain])
        writer.writerow(["Offset", offset])
    
    print(f"Calibration data saved to {filename}")


def main():
    sensor00 = VoltageRatioInput() #x-axis
    sensor01 = VoltageRatioInput() #y-axis	
    sensor02 = VoltageRatioInput() #z-axis
    
    #Set the serial number of the Phidget Voltage Ratio Input devices
    #You can find the serial number in the Phidget Control Panel
    
    # # Configuratiom Top Left
    
    # sensor00.setDeviceSerialNumber(481661) #replace with your actual serial number
    # sensor01.setDeviceSerialNumber(481661) #replace with your actual serial number	
    # sensor02.setDeviceSerialNumber(481661) #replace with your actual serial number
   
    # sensor00.setChannel(0) #x-axis -> channel 0 at the Phidget board
    # sensor01.setChannel(1) #y-axis -> channel 1 at the Phidget board
    # sensor02.setChannel(2) #z-axis -> channel 2 at the Phidget board 

    #Configuration Top Right
    
    """ sensor00.setDeviceSerialNumber(293698) #replace with your actual serial number
    sensor01.setDeviceSerialNumber(293698) #replace with your actual serial number
    sensor02.setDeviceSerialNumber(293698) #replace with your actual serial number
    sensor00.setChannel(0) #x-axis -> channel 3 at the Phidget board
    sensor01.setChannel(1) #y-axis -> channel 1 at the Phidget board
    sensor02.setChannel(2) #z-axis -> channel 2 at the Phidget board  """
    
    #Configuration Bottom Left

    sensor00.setDeviceSerialNumber(588832) #replace with your actual serial number
    sensor01.setDeviceSerialNumber(588832) #replace with your actual serial number
    sensor02.setDeviceSerialNumber(588832) #replace with your actual serial number
    sensor00.setChannel(0) #x-axis -> channel 0 at the Phidget board
    sensor01.setChannel(1) #y-axis -> channel 1 at the Phidget
    sensor02.setChannel(2) #z-axis -> channel 2 at the Phidget board
    
    #Configuration Bottom Right
    
    """ sensor00.setDeviceSerialNumber(293701) #replace with your actual serial number
    sensor01.setDeviceSerialNumber(293701) #replace with your actual serial number
    sensor02.setDeviceSerialNumber(293701) #replace with your actual serial number
    sensor00.setChannel(0) #x-axis -> channel 0 at the Phidget board
    sensor01.setChannel(1) #y-axis -> channel 1 at the Phidget
    sensor02.setChannel(2) #z-axis -> channel 2 at the Phidget board  """
    
    
    sensor00.setChannel(0) #x-axis -> channel 0 at the Phidget board
    sensor01.setChannel(1) #y-axis -> channel 1 at the Phidget
    sensor02.setChannel(2) #z-axis -> channel 2 at the Phidget board
    
    sensor00.openWaitForAttachment(5000)
    sensor01.openWaitForAttachment(5000)
    sensor02.openWaitForAttachment(5000)

    #Set the data interval for the application
    sensor00.setDataInterval(1000)
    sensor01.setDataInterval(1000)
    sensor02.setDataInterval(1000)
    
    voltages = []
    mean_voltages = []
    #weights = [10000,20000,30000] #in grams
    #weights = [10000 * 1/2, 20000 * 1/2, 30000 * 1/2] #in grams, for y-calibration with 30° angle
    weights = [10000 * np.sqrt(3)/2,20000 * np.sqrt(3)/2,30000 * np.sqrt(3)/2] #in grams
    #weights = [2000, 4000, 6000]  # in grams
    print("Lets start the calibration process")
    i = input("Which direction do you want to calibrate? (x,y,z) ")
    
    input("Hang first weight (10kg) on sensor and press enter to continue")
    get_voltage(sensor00, sensor01, sensor02,  voltages, mean_voltages, i)
    input("Hang second weight (20kg) on sensor and press enter to continue")
    get_voltage(sensor00, sensor01, sensor02,  voltages, mean_voltages, i)
    input("Hang third weight (30kg) on sensor and press enter to continue")
    get_voltage(sensor00, sensor01, sensor02,  voltages, mean_voltages, i)
    #input("Hang third weight (40kg) on sensor and press enter to continue")
    #get_voltage(sensor00, sensor01, sensor02, voltages, mean_voltages, i)



    print("Mean Voltages: ", mean_voltages)
    print("Weights: ", weights)

    gain, offset = np.polyfit(mean_voltages, weights, 1)  # 1 indicates a linear fit (degree 1)

    print(f"Calculated Gain (Slope): {gain}")
    print(f"Calculated Offset (Intercept): {offset}")

    plot_calibration_curve(mean_voltages, weights, gain, offset)

    #save_calibration_data(mean_voltages, gain, offset, filename="calibration_data_board_TL.csv")


    sensor00.close()
    sensor01.close()
    sensor02.close()
    

if __name__ == "__main__":
    main()