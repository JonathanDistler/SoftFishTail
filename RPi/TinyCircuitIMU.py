#################################
#	Imports and module names	#
#################################

#Import python packages for c types and
# os functions

#NEED to import TinyCircuit SAMD board module to use this in Arduino, select TinyZero


import os
import sys
sys.path.append('..')


# import pyserial tools to list ports
# and find where the Arduino is connected
import serial
import serial.tools.list_ports
from time import sleep

# import WiFi socket module for python server
import socket

USB = 1
WIFI = 2

COMMUNICATION = USB		# options: USB, WIFI

# Connection object for USB, if USB is used
serial_connection = 0
vtk_render_window = 0
last_quat = 0

board_id = -1

def get_orientation_data():
	if COMMUNICATION == USB:
		return serial_connection.readline().decode().split()
	elif COMMUNICATION == WIFI:
		# Quaterion string and char count: 0 -0.00 -0.00 -0.00 -0.00 [26 chars]
		return client.recv(26).decode().split()


#instead of function, can preset the code to hardcode the USB port (ie 8)
def connect_usb():
	print("\n*Searching for plugged in Arduino USB ports...")
	"""
	arduino_ports = list(serial.tools.list_ports.comports())
	port_number = 0

	print("*Found COM ports:")
	port_index = 0
	for port in arduino_ports:
		print(port_index, "\t", port)
		port_index = port_index + 1

	if not arduino_ports:
		print("*No Arduino boards were found to be plugged in - stopping debugger")
		exit()
	elif len(arduino_ports) > 1:
		print("*More than one COM port connection detected - enter number correlating to port to use (e.g. '0', '1', etc...)")
		try:
			port_number = int(input("*Enter number from above: "))
		except ValueError:
			print("*ERROR: Character entered was not a number - stopping debugger")
			exit()

	print("*Using board port \"", arduino_ports[port_number], "\" for IMU data stream")
	print("*Attempting to open port", arduino_ports[port_number].device)
	#print(arduino_ports[port_number].device)
	"""
	#hardcoded from Arduino IDE
	port="COM8"
	#serial_connection=serialSerial(USB8,115200)

	try:
		global serial_connection
		serial_connection = serial.Serial(port, 115200)
		#serial_connection = serial.Serial(arduino_ports[port_number].device, 115200)
		print("*Successfully opened device port\n")
		return 1
	except:
		print("\n*Could not open port: either the device was unplugged, another process is using the port, or permission denied - stopping debugger\n")
		exit()



def main():
    if COMMUNICATION == USB:
        print("\n*Looking for USB connection...\n")
        connect_usb()
    sleep(0.1)

    global board_id
    board_id = get_orientation_data()[0]
    print("BOARD ID: ", board_id)
    print("-----Orientation data-----")
    
    try:
        while True:
            data_1 = get_orientation_data()[1]
            data_2 = get_orientation_data()[2]
            print(f"Axis-1: {data_1}, Axis-2: {data_2}")
            # You can later call your rendering update here
            # e.g., update_scene_with_data(data)
            sleep(0.05)  # 20 Hz refresh rate
    except KeyboardInterrupt:
        print("\n*Exiting program (KeyboardInterrupt).")


if __name__ == '__main__':
	main()
