#!/usr/bin/python

"""
FUMEBOT CLIENT FOR RPI3B

This is the client to the FumeBot GUI program and is written to be run
on the Raspberry Pi side. This is the main python program that needs to
be executed. The program will spawn several threads for reading/sending
socket data and also for transmitting serial data to the Arduino and also
reading serial data.

Written by  :  Ajith Thomas
Date        :  5-4-2018
"""

import os
import time
import subprocess
from ConfigParser import ConfigParser
from FumeBotSerialComm import FumeBotSerial
from FumeBotSocketComm import FumeBotSocket
import RPi.GPIO as GPIO

# Status LED
RUNNING_LED=17
CONNECTED_LED=18
THREAD_RESTART_LED=27
RESET_AVR=22  # Pin that resets the Arduino if needed
MCU_RDY=23

# Configuration file for fumebot
CONFIG_FILE='bot_config.ini'
CONFIG_PATH='/home/pi/FumeBot/'
config_path=os.path.join(CONFIG_PATH,CONFIG_FILE)

# Host address and Ports for socket communication
host = '192.168.1.8'
data_port = 8090
capture_port = 8089

# Serial port address
serial_port = '/dev/serial0'
baud = 115200

# Video capture settings
width=640
height=480
fps=15
color=0  # 0 for color and 1 for grayscale
res_index=0  # Which index was used in the GUI to set the above resolution
enable_thermal=0  # Thermal camera enabler or disable

# Gas sensor thresholds
eCO2_thresh=500
TVOC_thresh=15

# Particle sensor thresholds
red=1000
green=1500
ir=2000

# SMS alert settings
sms_number="00000000000"  # Number to send the sms alert
sms_enabled=0 # Enable the SMS alert to be sent (0 disabled and 1 enabled)

# Wifi status and signal level
wifi_connected=True  # If the wifi is not connected this will be changed any way (This is the case only if nothing critical depends on this)
wifi_quality=0
wifi_signal=0
wifi_check_delay=2  # Check for a wifi connection interval (seconds)
prev_check_time=0
wifi_status_set=False

serial_rx_header_dict={  # Headers used when the microcontroller sents data to RPi
	'GAS':'g',
	'ENVIRONMENT':'e',
	'PARTICLE':'p',
	'CFG_REQ':'cr',  # This is used to update all the configuration of the microcontroller
	'SER_SET_ACK':'ak',  #Serial acknowledgement for setting the configuration
}

config_sent_to_uC=False;

ack_code_dict={  # Acnowledgement codes
	'GAS_THR':1,  # Used in both serial and socket communication (This code is sent only when uC returns an acknowledge message)
	'PAR_THR':2,  # Used in both serial and socket communication (This code is sent only when uC returns an acknowledge message)
	'SMS_SET':3,  # Used in both serial and socket communication (This code is sent only when uC returns an acknowledge message)
	'CON_SET':4,  # Only for socket
	'VID_SET':5,  # Only for socket
	'RST_ALM':6,  # Used in both serial and socket communication (This code is sent only when uC returns an acknowledge message)
}

serial_tx_header_dict={  # Used to transmit data to the microcontroller
	'MOVE':'m',
	'CAM_PT':'c',
	'GAS_THR':'gt',
	'PAR_THR':'pt',
	'SMS':'sm',
	'RST_ALM':'ra',  # This should also reset other variables like SMS sent when threshold reached boolean and other associated variables
}

socket_send_header_dict={  # This is send header used only for sockets
	'GAS':'G',
	'ENVIRONMENTAL':'E',
	'PARTICLE':'P',
	'WIFI':'W',
	'SOCK_SET_ACK':'AK',  # To say that the socket command to set was successful
	'GAS_THR_RET': 'GTR',
    'PAR_THR_RET': 'PTR',
    'SMS_RET': 'SR',
    'CAM_RET': 'CMR',
    'MCU_STAT': 'MS',
}

socket_receive_header_dict={  # This is the receive header used only for sockets
	'MOVE':'MV',
	'CAM_PT':'CPT',
	'CAM_SET':'CM',
	'COM_SET':'CO',
	'GAS_THR':'GT',
	'PAR_THR':'PT',
	'SMS':'SM',
	'REBOOT_RPI':'RRP',
	'RESET_UC':'RUC',
	'PWR_OFF_RPI':'PWR',
	'RESET_ALARM': 'RA',  # Reset the alarm state in the microcontroller
    'RETRIEVE': 'RET',  # This is used to retrieve settings (The headers will be the parameter)
}

# Microcontroller status
mcu_ready_now=False

def handle_serial_receive(rx_data):  # Function to handle the incoming serial data from microcontroller
	
	try:
		parsed_data=rx_data.split(",")

		header=parsed_data[0]  # First item is the header which determine the type of data

		if header == serial_rx_header_dict['GAS']:  # Gas sensor readings
			
			eCO2=int(parsed_data[1])
			TVOC=int(parsed_data[2])

			cmd=format_socket_comm((socket_send_header_dict['GAS']), [eCO2, TVOC])

			soc_comm.put_socket_data(cmd)

		elif header == serial_rx_header_dict['PARTICLE']: # Particle sensor readings
			
			red=int(parsed_data[1])
			green=int(parsed_data[2])
			ir=int(parsed_data[3])

			cmd=format_socket_comm(socket_send_header_dict['PARTICLE'], [red, green, ir])

			soc_comm.put_socket_data(cmd)

		elif header == serial_rx_header_dict['ENVIRONMENT']:  # Environmental sensor readings
			
			pressure=float(parsed_data[1])
			temperature=float(parsed_data[2])
			humidity=float(parsed_data[3])

			cmd=format_socket_comm(socket_send_header_dict['ENVIRONMENTAL'], [pressure, temperature, humidity])

			soc_comm.put_socket_data(cmd)

		elif header == serial_rx_header_dict['CFG_REQ']:  # This is for when the uC asks for the configuration data

			print("Microcontroller requested configuration data")

			send_configuration_data_to_uC()

			print("Microcontroller configuration data sent")

		elif header == serial_rx_header_dict['SER_SET_ACK']:  # If acknowledgement code is received

			# Get the code
			ack_code=int(parsed_data[1])

			# The acknowledgement for the socket commands to set parameters for the microcontroller are sent here if the uC responds with the correct code
			if ack_code == ack_code_dict['GAS_THR']:
				cmd=format_socket_comm(socket_send_header_dict['SOCK_SET_ACK'],[ack_code])
				soc_comm.put_socket_data(cmd)

				print("Gas sensor threshold set")

			elif ack_code == ack_code_dict['PAR_THR']:
				cmd=format_socket_comm(socket_send_header_dict['SOCK_SET_ACK'],[ack_code])
				soc_comm.put_socket_data(cmd)

				print("Particle sensor threshold set")

			elif ack_code == ack_code_dict['SMS_SET']:
				cmd=format_socket_comm(socket_send_header_dict['SOCK_SET_ACK'],[ack_code])
				soc_comm.put_socket_data(cmd)

				print("SMS settings applied")

			elif ack_code == ack_code_dict['RST_ALM']:
				cmd=format_socket_comm(socket_send_header_dict['SOCK_SET_ACK'],[ack_code])
				soc_comm.put_socket_data(cmd)

				print("Alarm reset in microcontroller")

	except:  # If any error occurs just ignore it
		print("Serial receive error!")
		pass

def handle_socket_receive(recv_data):  # Function for incoming socket data
	
	try:
		parsed_data=recv_data.split(",")

		header=parsed_data[0]

		if header == socket_receive_header_dict['MOVE']:  # For the movement
			
			move_dir=int(parsed_data[1])
			accl=int(parsed_data[2])
			brake=int(parsed_data[3])

			cmd=format_serial_comm(serial_tx_header_dict['MOVE'],[move_dir,accl,brake])

			# Send the move command over serial
			ser_comm.put_serial_data(cmd)

		elif header == socket_receive_header_dict['CAM_PT']:  # For camera panning and tilting
			
			pan_angle=int(parsed_data[1])
			tilt_angle=int(parsed_data[2])

			cmd=format_serial_comm(serial_tx_header_dict['CAM_PT'],[pan_angle,tilt_angle]) # Format the command

			#Send the camera pan and tilt servo command over serial
			ser_comm.put_serial_data(cmd)

		elif header == socket_receive_header_dict['CAM_SET']:  # For setting the camera resolution and FPS
			
			global width,height,fps,color,res_index,enable_thermal

			width=int(parsed_data[1])
			height=int(parsed_data[2])
			fps=int(parsed_data[3])
			color=int(parsed_data[4])  # Whether grayscale or full color
			res_index=int(parsed_data[5])
			enable_thermal=int(parsed_data[6])  # Enable or disable the thermal camera

			config.set('Video', 'width', width)
			config.set('Video', 'height', height)
			config.set('Video', 'fps', fps)
			config.set('Video', 'color', color)  # Default is full color
			config.set('Video', 'res_index', res_index)  # Index of the resolution currently being used
			config.set('Video', 'enable_thermal', enable_thermal)

			write_to_config_file()

			soc_comm.set_capture_settings(width, height, fps, color)  # Change the capture settings
			soc_comm.set_enable_thermal(enable_thermal,start_thread_now=True)  # Enable or disable the thermal camera

			# Send an acknowledge message back to server
			cmd=format_socket_comm(socket_send_header_dict['SOCK_SET_ACK'],[ack_code_dict['VID_SET']])
			soc_comm.put_socket_data(cmd)

			print("Camera settings applied")

		elif header == socket_receive_header_dict['COM_SET']:  # For setting the communication host and ports
			
			global host, data_port, capture_port

			host=str(parsed_data[1])
			data_port=int(parsed_data[2])
			capture_port=int(parsed_data[3])

			config.set('Connection', 'host', host)
			config.set('Connection', 'data_port', data_port)
			config.set('Connection', 'video_port', capture_port)

			write_to_config_file()

			# Send an acknowledge message back to server
			cmd=format_socket_comm(socket_send_header_dict['SOCK_SET_ACK'],[ack_code_dict['CON_SET']])
			soc_comm.put_socket_data(cmd)

			print("Connection settings applied")

		elif header == socket_receive_header_dict['GAS_THR']:  # Gas thresholds
			
			global eCO2_thresh,TVOC_thresh

			eCO2_thresh=int(parsed_data[1])
			TVOC_thresh=int(parsed_data[2])

			config.set('Gas_thresh', 'eco2', eCO2_thresh)
			config.set('Gas_thresh', 'tvoc', TVOC_thresh)

			write_to_config_file()

			# A serial command should be sent to set the thresholds
			cmd=format_serial_comm(serial_tx_header_dict['GAS_THR'],[eCO2_thresh,TVOC_thresh])
			ser_comm.put_serial_data(cmd)

			print("Gas sensor thresholds will be applied")
			# The acknowlede code is not sent here and is only sent if the microcontroller has set the values

		elif header == socket_receive_header_dict['PAR_THR']:  # Particle thresholds

			global red,green,ir

			red=int(parsed_data[1])
			green=int(parsed_data[2])
			ir=int(parsed_data[3])

			config.set('Particle_thresh', 'red', red)
			config.set('Particle_thresh', 'green', green)
			config.set('Particle_thresh', 'ir', ir)

			write_to_config_file()

			# A serial command should be sent to set the thresholds
			cmd=format_serial_comm(serial_tx_header_dict['PAR_THR'],[red,green,ir])
			ser_comm.put_serial_data(cmd)

			print("Particle sensor thresholds will be applied")
			# The acknowlede code is not sent here and is only sent if the microcontroller has set the values

		elif header == socket_receive_header_dict['SMS']:  # For setting the SMS

			global sms_number,sms_enabled

			sms_number=str(parsed_data[1])
			sms_enabled=int(parsed_data[2])

			config.set('SMS_alert', 'alert_number', sms_number)
			config.set('SMS_alert', 'alert_enabled', sms_enabled)

			write_to_config_file()

			# A serial command should be send to set the alert number
			cmd=format_serial_comm(serial_tx_header_dict['SMS'],[sms_number,sms_enabled])
			ser_comm.put_serial_data(cmd)

			print("SMS settings will be applied")
			# The acknowlede code is not sent here and is only sent if the microcontroller has set the values

		elif header == socket_receive_header_dict['REBOOT_RPI']:  # Reboots the Raspberry Pi
			
			print("Rebooting now.")

			time.sleep(1)
			subprocess.Popen(["sudo","reboot"])

		elif header == socket_receive_header_dict['RESET_UC']:  # Resets the microcontroller
			
			print("Resetting microcontroller")

			reset_microcontroller()

			print("Resetting done! Might take up to 10 seconds for microcontroller to come online again.")

		elif header == socket_receive_header_dict['PWR_OFF_RPI']:  # Power of the Raspberry Pi
			
			print("Powering down now.")

			time.sleep(1)
			subprocess.Popen(["sudo","poweroff"])

		elif header == socket_receive_header_dict['RESET_ALARM']:

			print("Resetting alarms")

			cmd=format_serial_comm(serial_tx_header_dict['RST_ALM'],[])
			ser_comm.put_serial_data(cmd)

		elif header == socket_receive_header_dict['RETRIEVE']:

			retreive_header=parsed_data[1]  # This is header for which we want to send the settings information back

			# Format ans send back the settings back
			if retreive_header == socket_receive_header_dict['CAM_SET']:

				cmd=format_socket_comm(socket_send_header_dict['CAM_RET'],[fps,color,res_index,enable_thermal])
				soc_comm.put_socket_data(cmd)

			elif retreive_header == socket_receive_header_dict['GAS_THR']:

				cmd=format_socket_comm(socket_send_header_dict['GAS_THR_RET'],[eCO2_thresh,TVOC_thresh])
				soc_comm.put_socket_data(cmd)

			elif retreive_header == socket_receive_header_dict['PAR_THR']:

				cmd=format_socket_comm(socket_send_header_dict['PAR_THR_RET'],[red,green,ir])
				soc_comm.put_socket_data(cmd)

			elif retreive_header == socket_receive_header_dict['SMS']:

				cmd=format_socket_comm(socket_send_header_dict['SMS_RET'],[sms_number,sms_enabled])
				soc_comm.put_socket_data(cmd)
	
	except:  # If any error occurs just pass
		print("Socket receive error!")
		pass

def format_socket_comm(cmd_type, data_list):  # Function to format socket commands
	start_marker = "$"
	end_marker = "#"

	cmd_string = start_marker + cmd_type

	for i in range(len(data_list)):
		cmd_string = cmd_string + "," + str(data_list[i])

	return cmd_string + end_marker


def format_serial_comm(cmd_type,data_list):  # Function to format serial commands
	cmd_string=cmd_type

	for i in range(len(data_list)):
		cmd_string = cmd_string + "," + str(data_list[i])

	return cmd_string+","+"\n"  # Add a last comma and newline character


def make_bot_config_file():  # Function to make a configuration file
	print("Making bot configuration file")

	config.add_section('Connection')
	config.set('Connection','host',host)
	config.set('Connection','data_port',data_port)
	config.set('Connection','video_port',capture_port)

	config.add_section('Serial')
	config.set('Serial','serial_port',serial_port)
	config.set('Serial','baud',baud)

	config.add_section('Video')
	config.set('Video','width',width)
	config.set('Video','height',height)
	config.set('Video','fps',fps)
	config.set('Video','color',color)  # Default is full color
	config.set('Video','res_index',res_index)  # Index of the resolution currently being used
	config.set('Video','enable_thermal',enable_thermal)  # Enables or disables thermal camera on start up

	config.add_section('Gas_thresh')
	config.set('Gas_thresh','eco2',eCO2_thresh)
	config.set('Gas_thresh','tvoc',TVOC_thresh)

	config.add_section('Particle_thresh')
	config.set('Particle_thresh','red',red)
	config.set('Particle_thresh','green',green)
	config.set('Particle_thresh','ir',ir)

	config.add_section('SMS_alert')
	config.set('SMS_alert','alert_number',sms_number)
	config.set('SMS_alert','alert_enabled',sms_enabled)

	write_to_config_file()  # Write the above to a new configuration file


def write_to_config_file():
	with open(config_path,'w') as configfile:
		config.write(configfile)


def update_defaults_from_config_file():  # Get configuration from the file and update the variables
	global host,data_port,capture_port

	host=config.get('Connection','host')
	data_port=config.getint('Connection','data_port')
	capture_port=config.getint('Connection','video_port')

	global serial_port, baud

	serial_port=config.get('Serial','serial_port')
	baud=config.getint('Serial','baud')

	global width,height,fps,color,res_index,enable_thermal

	width=config.getint('Video','width')
	height=config.getint('Video','height')
	fps=config.getint('Video','fps')
	color=config.getint('Video','color')
	res_index=config.getint('Video','res_index')
	enable_thermal=config.getint('Video','enable_thermal')

	global eCO2_thresh, TVOC_thresh

	eCO2_thresh=config.getint('Gas_thresh','eco2')
	TVOC_thresh=config.getint('Gas_thresh','tvoc')

	global red,green,ir
		
	red=config.getint('Particle_thresh','red')
	green=config.getint('Particle_thresh','green')
	ir=config.getint('Particle_thresh','ir')

	global sms_number,sms_enabled

	sms_number=config.get('SMS_alert','alert_number')
	sms_enabled=config.getint('SMS_alert','alert_enabled')

 
def get_wifi_signal_strength():  # Function to get the wifi strength (Returns signal quality and signal strenght)
	pw=subprocess.Popen(["iwconfig","wlan0"],stdout=subprocess.PIPE)
	pg=subprocess.Popen(["grep","Link Quality"],stdin=pw.stdout,stdout=subprocess.PIPE)
	output,_=pg.communicate()
	output=output.strip().replace("=",",").replace("  ",",").replace(" ",",").replace("/",",") 
	output_list=output.split(",")  # Parsed output list
	
	try: # Try to calculate the link quality in percentage
		link_quality=(float(output_list[2])/float(output_list[3]))*100.0
		signal_level=int(output_list[6])
	except IndexError:  # If the wifi connection is lost an index error will occur
		return None,None

	return link_quality,signal_level

def is_wifi_connected():  # Function to check whether the wifi is connected or not
	
	p=subprocess.Popen(["ip","route"],stdout=subprocess.PIPE)
	output,_=p.communicate()
	if output != "": # If it is not an empty string then wifi is connected
		return True
	else:
		return False

def send_wifi_info():  # Function to send the wifi information
	cmd=format_socket_comm(socket_send_header_dict['WIFI'],[wifi_quality,wifi_signal])

	soc_comm.put_socket_data(cmd)  # Send the wifi information

def set_program_status_led():  # Function to set the status led

	global wifi_status_set  # Tell to use the global variable
	
	if wifi_connected:
		blink_running_pwm.stop()  # Stop blinking the red led for wifi disconnection status
		wifi_status_set=False
		
		if soc_comm.is_data_socket_connected() and soc_comm.is_capture_socket_connected():  # If both sockets are connected
			blink_connect_pwm.stop()
			GPIO.output(CONNECTED_LED,GPIO.HIGH)
			GPIO.output(RUNNING_LED,GPIO.LOW)

		elif soc_comm.is_data_socket_connected() or soc_comm.is_capture_socket_connected():  # If any one of the socket is connected
			GPIO.output(RUNNING_LED,GPIO.LOW)
			blink_connect_pwm.start(50)  # Start blinking the green led since one of the port is connected

		else:  # If none of the socket is connected
			blink_connect_pwm.stop()
			GPIO.output(CONNECTED_LED,GPIO.LOW)
			GPIO.output(RUNNING_LED,GPIO.HIGH)

	elif not wifi_connected and not wifi_status_set:  # Don't enter after the wifi dissconnection is noticed
		GPIO.output(RUNNING_LED,GPIO.LOW)
		GPIO.output(CONNECTED_LED,GPIO.LOW)
		blink_running_pwm.start(50)  # Show that the wifi is disconnected
		wifi_status_set=True  # The wifi disconnection status is now set

def set_program_soft_restart_led(rst_done):  # Lights the blue led when the threads are restarted (Bascially a soft restart)
	GPIO.output(CONNECTED_LED,GPIO.LOW)
	GPIO.output(RUNNING_LED,GPIO.LOW)

	if rst_done is True:
		GPIO.output(THREAD_RESTART_LED,GPIO.HIGH)
	else:
		GPIO.output(THREAD_RESTART_LED,GPIO.LOW)

def send_configuration_data_to_uC():  # Function to send the configuration data to be sent to uC
	# Set the gas threshold first
		cmd=format_serial_comm(serial_tx_header_dict['GAS_THR'],[eCO2_thresh,TVOC_thresh])
		ser_comm.put_serial_data(cmd)

		# Set the particle threshold
		cmd=format_serial_comm(serial_tx_header_dict['PAR_THR'],[red,green,ir])
		ser_comm.put_serial_data(cmd)

		# Set the SMS number and whetheer its is enabled
		cmd=format_serial_comm(serial_tx_header_dict['SMS'],[sms_number,sms_enabled])
		ser_comm.put_serial_data(cmd)

def reset_microcontroller():  # Function to reset the AVR microcontroller
	GPIO.output(RESET_AVR,GPIO.HIGH)  # Resets the microcontroller
	time.sleep(1)  # Reset is kept active low for 1 second
	GPIO.output(RESET_AVR,GPIO.LOW)  # Let the reset pin go high

def send_microcontroller_status(ready=False):  # Function to say the microcontroller status
	status=0
	if ready == True:
		status=1
	elif ready == False:
		status=0

	cmd=format_socket_comm(socket_send_header_dict['MCU_STAT'],[status])
	soc_comm.put_socket_data(cmd)


if __name__ == '__main__':

	print("FumeBot comms active!")

	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)  # Set it to false so warnings don't show up

	GPIO.setup(RUNNING_LED,GPIO.OUT)
	GPIO.setup(CONNECTED_LED,GPIO.OUT)
	GPIO.setup(THREAD_RESTART_LED,GPIO.OUT)
	GPIO.setup(RESET_AVR,GPIO.OUT)  # This is pin is used to reset the Arduino
	GPIO.setup(MCU_RDY,GPIO.IN)

	blink_running_pwm=GPIO.PWM(RUNNING_LED,1)
	blink_connect_pwm=GPIO.PWM(CONNECTED_LED,1)

	config=ConfigParser()

	try:
		config.readfp(open(config_path))
	except IOError:
		make_bot_config_file()

	# Read the configuration file
	config.read(config_path)

	# The config file is read before any initialization of any sort is done
	update_defaults_from_config_file()  # Get the values from the config file

	# Creating the socket object
	soc_comm=FumeBotSocket(host=host,data_port=data_port, capture_port=capture_port)  # For socket communication
	soc_comm.set_capture_settings(width,height,fps,color)  # Set the capture settings for the camera
	soc_comm.set_enable_thermal(enable_thermal,start_thread_now=False)  # Enable or disable the thermal camera

	# For serial communication
	ser_comm=FumeBotSerial(serial_port=serial_port,baud=baud)

	# Start the socket thread
	soc_comm.start_soc_read_thread()  # The socket read thread is started first
	soc_comm.start_soc_send_thread()
	soc_comm.start_capture_socket_thread()  # Start the capture socket thread
	soc_comm.start_capture_thread()  # Start the camera thread
	soc_comm.start_thermal_capture_thread()

	# Start serial threads
	ser_comm.start_serial_read_thread()  # The serial read thread is started first
	ser_comm.start_serial_send_thread()

	while True:  # Main loop starts here

		set_program_status_led()  # Set the LED status

		# Send the configuration on data connection
		if soc_comm.is_data_socket_connected() and not config_sent_to_uC and mcu_ready_now:
			print("Configuration sent to microcontroller")
			send_configuration_data_to_uC()
			send_microcontroller_status(ready=mcu_ready_now)
			config_sent_to_uC=True

		elif soc_comm.is_data_socket_connected() is False:
			config_sent_to_uC=False
		
		# Check to see if the microcontroller is ready
		if GPIO.input(MCU_RDY):
			if not mcu_ready_now:  # MCU is ready from not being ready
				print("Microcontroller is ready")
				send_microcontroller_status(ready=True)
				mcu_ready_now=True
		
		else:
			if mcu_ready_now:  # MCU is not ready from being ready
				print("Microcontroller is not ready")
				send_microcontroller_status(ready=False)
				config_sent_to_uC=False  # Set that the microcontroller needs it configuration 
				mcu_ready_now=False


		# First check for a wifi connection (This is repeated according to the delay)
		if (time.time()-prev_check_time) > wifi_check_delay:
			if is_wifi_connected() is True:
				wifi_connected=True
				wifi_quality,wifi_signal=get_wifi_signal_strength()
				send_wifi_info()  # Function to send the wifi information
			else:
				wifi_connected=False
				soc_comm.set_sockets_as_not_connected()  # Puts the thread in the try to connect state
				print("Wifi not connected!")

			prev_check_time=time.time()

		# This is main data reception, handling and execution
		if soc_comm.is_socket_data_available():  # If any socket data is available
			data=soc_comm.get_socket_data()
			handle_socket_receive(data)

		if ser_comm.is_serial_data_available():  # If any serial data is available 
			data=ser_comm.get_serial_data()
			handle_serial_receive(data)

		#This is one entered if the threads have crashed for some reason (Only the socket part is restarted)
		if not soc_comm.is_socket_threads_running():  # If all the threads closed
			print("Socket threads have unexpectedly stopped! Attempting to restart...")

			set_program_soft_restart_led(True)

			update_defaults_from_config_file()

			soc_comm.stop_all_socket_threads()
			
			soc_comm.__init__(host,data_port,capture_port)  # Reinitialize the socket
			soc_comm.set_capture_settings(width,height,fps,color)

			time.sleep(5)  # Sleep for 5 seconds

			print("Restarting threads now!")

			# Start the socket thread
			soc_comm.start_soc_read_thread()  # The socket read thread is started first
			soc_comm.start_soc_send_thread()
			soc_comm.start_capture_socket_thread()  # Start the capture socket thread
			soc_comm.start_capture_thread()  # Start the camera thread
			soc_comm.start_thermal_capture_thread()

			print("Threads Restarted!")

			set_program_soft_restart_led(False)

	GPIO.cleanup()






