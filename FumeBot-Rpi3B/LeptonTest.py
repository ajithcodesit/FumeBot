#!/bin/usr/python

import cv2
import numpy as np
from Lepton3CapLib import Lepton3

lep=Lepton3(device="/dev/spidev0.0",speed=25000000,delay=65535,debug=True)

img_array=np.zeros((120,160,1),dtype=np.uint32)

# print("Opening Lepton 3")
# lep.openLepton3()

# print("Capturing thermal image...")
# lep.captureLepton3(img_array)

# img_array=np.uint16(img_array)
# cv2.normalize(img_array,img_array,0,65535,cv2.NORM_MINMAX)  # Extending the contrast
# np.right_shift(img_array,8,img_array)
# final_img_arr=np.uint8(img_array)

# #_, therm_jpeg=cv2.imencode('.jpg',final_img_arr)  # This will give a long line image which has the pixels in sequence

# print("Writing image to disk...")
# cv2.imwrite('Output.jpg',final_img_arr)
# print("Done")

# print("Closing Lepton 3")
# lep.closeLepton3()

# This is only a test to see whether the data from the thread is being read properly
lep.openLepton3stream()

while True:
	lep.readLepton3stream(img_array)

	#print(img_array)

lep.closeLepton3stream()



