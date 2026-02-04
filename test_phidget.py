from Phidget22.Devices.VoltageRatioInput import *

ch = VoltageRatioInput()
ch.setDeviceSerialNumber(588832)
ch.setChannel(0)
ch.openWaitForAttachment(5000)

print("Attached! VR =", ch.getVoltageRatio())
ch.close()
