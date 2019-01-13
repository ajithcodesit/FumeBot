#!/bin/usr/python

import ctypes

class Lepton3(object):

	def __init__(self,device="/dev/spidev0.0",speed=16000000, delay=65535,debug=True):
		super(Lepton3,self).__init__()

		self.dev_name=device.encode('utf-8')  # Encoded as UTF-8 to be sent to the C program
		self.dev_speed=speed
		self.dev_delay=delay
		self.enable_debug=debug

		self.lep3CapLib=ctypes.CDLL("/home/pi/FumeBot/libLep3cap.so")  # Load the shared object library

		# Setup Lepton 3 SPI device name and speed
		self.lep3CapLib.setup_lepton3(ctypes.c_char_p(self.dev_name),ctypes.c_uint32(self.dev_speed),
									  ctypes.c_uint16(self.dev_delay),ctypes.c_bool(self.enable_debug))

	def openLepton3(self):  # Opens the file descriptor
		self.lep3CapLib.open_lepton3()

	def captureLepton3(self,img_arr):  # To get the images from the camera (A numpy array of (height,width,channel))
		res_img_arr=img_arr.ctypes.data_as(ctypes.POINTER(ctypes.c_int))
		self.lep3CapLib.lepton3_capture(res_img_arr)

	def closeLepton3(self):  # Closes the file descriptor
		self.lep3CapLib.close_lepton3()

	def openLepton3stream(self):
		self.lep3CapLib.open_lepton3_stream()  # This will start the thread as well

	def readLepton3stream(self,img_arr):
		res_img_arr=img_arr.ctypes.data_as(ctypes.POINTER(ctypes.c_int))
		self.lep3CapLib.read_lepton3_stream(res_img_arr)

	def closeLepton3stream(self):
		self.lep3CapLib.close_lepton3_stream();  # This will stop the thread as well also close the file descriptor
