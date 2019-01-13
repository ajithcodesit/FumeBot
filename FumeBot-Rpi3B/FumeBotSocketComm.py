#!/usr/bin/python2

"""
FUMEBOT SOCKET COMMUNICATION

This a python module for socket connection to computer acting as
the host from the Raspberry Pi side. The module produces two types
of connection by connecting to the same host on different ports.
One port is used to transmit image data from the camera and only have
an associated socket to send the image, the other is the data port which
is used for receiving and sending data to the host computer. All of the
above are threaded. Data from the thermal camera is also sent through the 
same video port. (Known issue thermal camera is extremely glitchy.)

Written by  :  Ajith Thomas
Date        :  5-4-2018
"""

import socket
import time
from queue import Queue
from threading import Thread
import cv2
import numpy as np 
from Lepton3CapLib import Lepton3


class FumeBotSocket(object):

    q_data_r=Queue(maxsize=50)  # Queue that stores the newly arrived data
    q_data_s=Queue(maxsize=50)  # Queue that stores the data to be send
    q_frame=Queue(maxsize=100)  # Queue that stores the image frames
    q_therm_frame=Queue(maxsize=50)  # Queue that stores the thermal images

    q_frame_max_limit=10  # The maximum number of frame lag allowed is 10
    q_frame_therm_max_limit=10  # The limit for thermal camera
    frame_drop_count=0  # Count the number of timer frames were dropped

    jpeg_frame_bytes = b''  # The jpeg bytes

    def __init__(self,host='localhost',data_port=8090,capture_port=8089):
        super(FumeBotSocket, self).__init__()

        self.host=host
        self.cap_port=capture_port
        self.data_port=data_port

        # Camera device and settings
        self.cap_device=0
        self.cap=cv2.VideoCapture(self.cap_device)  # Capture the video from the first device
        
        self.cap_width=640
        self.cap_height=480
        self.cap_fps=25
        self.use_grayscale=False
        self.cap_prop_changed=True

        # Thermal camera settings
        self.thermal_enabled=False  # Thermal camera disabled by default
        self.thermal_cap_device="/dev/spidev0.0"
        self.thermal_device_speed=32000000
        self.thermal_device_delay=65535
        self.lepton_buf=np.zeros((120,160,1), dtype=np.uint32)  # Height, Width and Color (Buffer for the thermal image)

        self.lep=Lepton3(device=self.thermal_cap_device,speed=self.thermal_device_speed,delay=self.thermal_device_delay,debug=False)

        self.SOCKET_TIMEOUT=10.0

        # Creating the object of the socket class data transmission
        self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.data_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.data_socket.settimeout(self.SOCKET_TIMEOUT)
        self.data_socket_connected=False  # Tells whether the socket is connected
        # Timeouts are used but is not required

        # Creating the object of the socket class for image transmission
        self.cap_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.cap_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.cap_socket.settimeout(self.SOCKET_TIMEOUT)
        self.capture_socket_connected=False
        # Timeouts are used but is not required

        # For reading data from the socket using a thread
        self.data_read_thread=Thread(target=self.read_socket_data, args=(self.q_data_r,))
        self.data_read_thread.setDaemon(True)
        self.read_thread_active=False

        # For sending data through the socket using a thread
        self.data_send_thread=Thread(target=self.send_socket_data, args=(self.q_data_s,))
        self.data_send_thread.setDaemon(True)
        self.send_thread_active=False

        # For capturing video on a different thread
        self.cap_thread=Thread(target=self.capture_video, args=(self.q_frame, self.q_frame_max_limit,))
        self.cap_thread.setDaemon(True)
        self.cap_thread_active=False

        # For capturing the thermal video on a different thread
        self.cap_therm_thread=Thread(target=self.capture_thermal_video, args=(self.q_therm_frame,self.q_frame_therm_max_limit,))
        self.cap_therm_thread.setDaemon(True)
        self.cap_threm_thread_active=False

        # For sending the capture data through the capture socket
        self.cap_soc_thread=Thread(target=self.send_capture_frame,args=(self.q_frame,))
        self.cap_soc_thread.setDaemon(True)
        self.soc_cap_thread_active=False

    def start_soc_read_thread(self):  # Start this first
        if not self.read_thread_active:
            self.read_thread_active=True
            self.data_read_thread.start()

    def start_soc_send_thread(self):  # Start when ever
        if not self.send_thread_active:
            self.send_thread_active=True
            self.data_send_thread.start()

    def start_capture_thread(self):  # Start when ever
        if not self.cap_thread_active:
            self.cap_thread_active=True
            self.cap_thread.start()  # Start the capture process for the camera

    def start_thermal_capture_thread(self):  # Start when ever
        if not self.cap_threm_thread_active and self.thermal_enabled:
            self.cap_threm_thread_active=True
            self.cap_therm_thread.start()  # Start the capture for the thermal camera

    def start_capture_socket_thread(self):  # Start this first
        if not self.soc_cap_thread_active:
            self.soc_cap_thread_active=True
            self.cap_soc_thread.start()  # Start the send capture data thread

    def start_all_socket_thread(self):  # Function to start all the socket threads in order
        self.start_soc_read_thread()
        self.start_soc_send_thread()
        self.start_capture_socket_thread()
        self.start_capture_thread()
        self.start_thermal_capture_thread()

    def stop_all_socket_threads(self):  # Function to stop all the thread
        self.send_thread_active=False
        self.read_thread_active=False
        self.cap_thread_active=False
        self.soc_cap_thread_active=False
        self.cap_thread_active=False

    def join_all_threads(self):  # Function to join all the threads to the main thread
        self.data_read_thread.join(0)
        self.data_send_thread.join(0)
        self.cap_thread.join(0)
        self.cap_soc_thread.join(0)
        self.cap_therm_thread.join(0)

    def is_socket_threads_running(self):  # Check if all the threads are started
        if self.send_thread_active and self.read_thread_active and self.cap_thread_active and self.soc_cap_thread_active:
            if self.cap_threm_thread_active or not self.thermal_enabled:
                return True  # If all threads are running
            else:
                return False # The thermal capture thread should be running as it is enabled
        else:
            return False  # If all the thread are not running

    def is_data_socket_connected(self):  # Check the if the data socket is connected
        return self.data_socket_connected

    def is_capture_socket_connected(self):  # Check if the capture socket is connected
        return self.capture_socket_connected

    def set_sockets_as_not_connected(self):  # Function to reset the socket connection status (Use full in wifi disconnection case)
        #This hopefully puts the theads into the try to connect sockets state
        self.data_socket_connected=False
        self.capture_socket_connected=False

    def connect_data_socket(self):  # Function to connect the data socket
        """
        This function should be called before the received code used to transmit.
        When the socket connects this enables the send socket data thread (The thread is
        always running but doesn't do anything because the boolean data socket connected
        prevents entry)
        """

        while not self.data_socket_connected: # As long as the socket is not connected
            # If the connection is closed we have create a new socket (Just create the socket again)
            self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.data_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.data_socket.settimeout(self.SOCKET_TIMEOUT)

            try:  # Try to connect
                self.data_socket.connect((self.host, self.data_port))
                self.data_socket_connected=True
                print("Data socket connection established")
                break  # Break out of this while loop to do read/send

            except socket.error as e:  # Capture any exception
                if e.errno == socket.errno.ECONNREFUSED or socket.timeout:
                    print("Data socket connect error: "+str(e))
                    self.data_socket_connected=False
                    self.data_socket.close()
                    time.sleep(1)
                    continue  # Ignored

                else:  # Some serious error has happened
                    self.data_socket.close()  # Closing the socket
                    print("Data socket unhandled connect error: "+str(e))

    def connect_capture_socket(self):
        """
        This function should be used preferably in the camera capture socket
        send function as this also controls the camera capture thread. This enable
        the camera start taking frames in and putting it in the queue.
        """

        while not self.capture_socket_connected:  # As long as the socket is not connected
            # If the connection is closed we have create a new socket (Just create the socket again)
            self.cap_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.cap_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.cap_socket.settimeout(self.SOCKET_TIMEOUT)

            try:  # Try to connect
                self.cap_socket.connect((self.host, self.cap_port))
                self.capture_socket_connected=True
                print("Capture socket connection established")
                break   # Break out of this while loop to do read/send

            except socket.error as e:  # Capture any exception
                if e.errno == socket.errno.ECONNREFUSED or socket.timeout:
                    print("Capture socket connect error: "+str(e))
                    self.capture_socket_connected=False
                    self.cap_socket.close()
                    time.sleep(1)
                    continue  # Ignored

                else:  # Some serious error has happened
                    self.cap_socket.close()  # Socket is closed
                    print("Capture socket connect unhandled error: "+str(e))

    '''
    TO FUTURE ME:
    
    Basic working of the send or receive socket is like this: socket tries to read or send data, if the socket
    is not connected it just times out and is captured by the except statement as a unhandled exception case and 
    is ignored but the socket connected boolean is set to False. This causes the connect socket function to operate and
    tries to connect with the server. If the connection is established the connect socket function while loop
    is broken out and enters the while loop to read/send data. When the connection is closed by the user
    a Shutdown is first done, This causes a bad pipe error exception which causes it to return back to the
    connect socket function which tries to connect to the server again.
    
    [Some of the code might seem like redundant but its just done to see what is really happening instead of
    concatenating everything which might be hard to unravel upon first inspection.] 
    '''

    """################################################DATA STREAMS################################################"""

    def send_socket_data(self,qd):  # Function to send socket data

        while self.send_thread_active is True:  # Loop only if the thread is set to be active

            if self.data_socket_connected is True:  # If the data socket is connected then enter

                if qd.qsize() > 0:  # If there is data available in the queue
                    try:
                        send_str_data=qd.get()  # Get the queued data
                        byte_data = send_str_data.encode('utf-8')  # Encode it to utf-8
                        self.data_socket.sendall(byte_data)  # Send it over the socket

                    except socket.error as e:
                        if e.errno == socket.errno.ECONNRESET or socket.timeout:
                            print("Socket send data error: " + str(e))
                            self.data_socket_connected = False  # Data socket no longer connected
                            self.data_socket.close()  # This closes the underlying file descriptor
                            continue  # Ignored

                        else:
                            self.data_socket_connected=False
                            self.data_socket.close()  # This closes the underlying file descriptor
                            print("Send data error unhandled: "+str(e))

        self.send_thread_active=False
        print("Send data thread stopped!")

    def read_socket_data(self,qd):  # Function to read data from the socket
        byte_data=b''
        string_decoded=""

        while self.read_thread_active is True:
            self.connect_data_socket()  # Connect the data socket

            try:
                self.data_socket.sendall(b'\x00')  # Send a null byte to check if connection is still open
                byte_data+=self.data_socket.recv(64)  # Read up to 64 bytes

                start_marker=byte_data.find(b'$')
                end_marker=byte_data.find(b'#')

                if start_marker != -1 and end_marker != -1:
                    data_bytes=byte_data[start_marker+1:end_marker]  # The start and end markers are stripped
                    byte_data=byte_data[end_marker+1:]  # Bytes set to after the end marker

                    string_decoded=data_bytes.decode('utf-8')  # Decode the byte string

                    if not qd.full():
                        qd.put(string_decoded)  # Put the string into the queue

                    else:
                        print("FIFO full!")

            except socket.error as e:
                if e.errno == socket.errno.ECONNRESET or socket.timeout:
                    print("Socket read data error: "+str(e))
                    self.data_socket_connected=False  # Data socket no longer connected
                    self.data_socket.close()  # Close the file descriptor for the socket (New socket has to be created)
                    continue  # Ignored

                else:
                    self.data_socket_connected=False
                    self.data_socket.close()  # This closes the underlying file descriptor
                    print("Read data unhandled error: "+str(e))

        self.read_thread_active=False
        print("Read data thread stopped!")

    def get_socket_data(self):  # Function to the read the socket data from the queue
        if self.q_data_r.qsize() > 0:
            data=self.q_data_r.get()
            print("Data received: "+str(data))
            return data

    def is_socket_data_available(self):  # Function to check whether data is available in the queue
        if self.q_data_r.qsize() > 0:
            return True
        else:
            return False

    def put_socket_data(self,data_str):  # Function to send socket data byt putting it in the queue
        if not self.q_data_s.full() and self.data_socket_connected:  # If the queue is not full
            self.q_data_s.put(data_str)  # Put the data in the queue to be sent

    """##############################################VIDEO CAPTURE##############################################"""

    def set_capture_settings(self, width, height, fps, color):  # Function to change the capture settings
        self.cap_width=width
        self.cap_height=height
        self.cap_fps=fps

        if color == 0:  # Color frames will be sent
            self.use_grayscale=False
        elif color == 1:  # Grayscale frame will be sent
            self.use_grayscale=True

        self.cap_prop_changed=True  # The settings have now changed so set to true

    def capture_video(self, qf, qf_limit):  # Function to capture video and encoding to JPEG
        while self.cap_thread_active is True:

            if self.capture_socket_connected:  # If the capture socket is connected then start the camera

            	self.cap.open(self.cap_device)  # Try to open the capture device

                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cap_width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cap_height)
                self.cap.set(cv2.CAP_PROP_FPS, self.cap_fps)
                self.cap_prop_changed=False  # We have already set the capture properties
                time.sleep(1)  # Allow the camera to get ready and apply the settings
                print("Capture opened")

                while self.cap.isOpened() and self.capture_socket_connected is True:

                    if self.cap_prop_changed is True:  # If the capture properties changed
                        # Set the capture properties
                        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cap_width)
                        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cap_height)
                        self.cap.set(cv2.CAP_PROP_FPS, self.cap_fps)
                        print("Capture properties changed")
                        print(str(self.cap_width)+" | "+str(self.cap_height)+" | "+str(self.cap_fps))
                        self.cap_prop_changed=False
                        time.sleep(1)  # Wait for some time to apply the settings before reading from the camera

                    retval, frame = self.cap.read()  # Get the frames from the normal camera

                    if retval is False:  # If the frame is not able to be retreived from the camera
                    	print("Camera read failed")
                    	self.cap.release() 
                    	time.sleep(1)
                    	print("Trying to recover camera capture...")
                    	break  # Exit the capture loop and try to recover automatically from the error (Mainly the VIDEO_IO error)

                    thermal_jpeg_bytes=b''

                    if self.thermal_enabled is True:  # If the thermal camera is enabled
                        thermal_jpeg_bytes=self.get_thermal_frame_bytes()  # Get the thermal frames
                    else:
                        thermal_jpeg_bytes=b''  # Else send frame saying thermal camera is enabled

                    if self.use_grayscale is True:  # If use of grayscale is enabled
                        frame=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)  # Convert to grayscale

                    _, jpeg = cv2.imencode('.jpg', frame)  # Encode as JPEG
                    jpeg_bytes = b'\RS'+jpeg.tobytes()+b'\RE'  # Convert to a byte string with modifed start and end markers

                    total_jpeg_bytes=jpeg_bytes+thermal_jpeg_bytes  # Combination of the normal and thermal frames

                    if not qf.full():  # If the queue is not full
                        if qf.qsize() < qf_limit: # If the queue size is with in limits
                            qf.put(total_jpeg_bytes)  # Put it into the queue
                        else:  # Drop the frames
                            qf.get()  # This reduces the queue size
                            self.frame_drop_count+=1

                self.cap.release()

        self.cap_thread_active=False
        print("Camera capture stopped!")

    def set_enable_thermal(self,state,start_thread_now=False):  # Function to enable or disabled the thermal camera (This only stops the thread from not executing the commands for the thermal camera)

        if state == 0:
            self.thermal_enabled = False
            self.cap_threm_thread_active = False
            print("Thermal camera is disabled")
        elif state == 1:
            self.thermal_enabled = True
            print("Thermal camera is enabled")
            if start_thread_now is True:
                self.start_thermal_capture_thread()
                print("Thermal camera capture started as it was enabled")


    def capture_thermal_video(self,qf,qf_limit):  # Function to caputure the thermal video
        
        while self.cap_threm_thread_active is True:
            
            if self.capture_socket_connected:

                self.lep.openLepton3()  # open the device

                while self.capture_socket_connected and self.thermal_enabled:

                    self.lep.captureLepton3(self.lepton_buf)
                    
                    therm_frame_uint16_t=np.uint16(self.lepton_buf)
                    cv2.normalize(therm_frame_uint16_t,therm_frame_uint16_t,0,65535,cv2.NORM_MINMAX)  # Extending the contrast
                    np.right_shift(therm_frame_uint16_t,8,therm_frame_uint16_t)  # Fitting the above into a 8 Bit space
                    therm_frame_uint8_t=np.uint8(therm_frame_uint16_t)  # Cast the elements to uint8 type
                    _, therm_jpeg=cv2.imencode('.jpg',therm_frame_uint8_t)  # Encode in jpeg format

                    '''
                    Since both the normal camera and thermal camera uses the same port to send the image
                    as JPEG images, we need to distinguish between them, therefore the start and the end 
                    marker for the thermal JPEG frame will be modified. 

                    Start marker will be b'\TS\xff\xd8' and end marker will be b'\xff\xd9\TE'
                    '''

                    therm_jpeg_bytes=b'\TS'+therm_jpeg.tobytes()+b'\TE' # Convert to a modifed byte string

                    if not qf.full(): # If the queue for thermal frames is not full
                        if qf.qsize() < qf_limit:  # If the limit is not reached as well
                            qf.put(therm_jpeg_bytes)
                        else:
                            qf.get() # Take a frame off the queue

                self.lep.closeLepton3()  # Close Lepton 3

        self.cap_threm_thread_active=False
        print("Thermal camera capture stopped!")

    def get_thermal_frame_bytes(self):  # Function to return the thermal camera image bytes
        if self.q_therm_frame.qsize() > 0:
            data_bytes=self.q_therm_frame.get()
            return data_bytes
        else:
            return b''


    def send_capture_frame(self,qf):  # Function to send frames on a different thread

        while self.soc_cap_thread_active is True:
            self.connect_capture_socket()  # Connect to the capture socket

            if not qf.empty():  # If queue is not empty
                try:
                    jpeg_frame_bytes = qf.get()  # Get the frame byte strings from the queue
                    self.cap_socket.sendall(jpeg_frame_bytes)  # Send the capture data

                except socket.error as e:
                    if e.errno == socket.errno.ECONNRESET or socket.timeout:
                        print("Socket capture send error "+str(e))
                        self.capture_socket_connected=False  # Capture socket no longer connected
                        self.cap_socket.close()  # Close the socket file descriptor
                        continue  # Ignored

                    else:
                        self.capture_socket_connected=False
                        self.cap_socket.close()
                        print("Capture send unhandled error: "+str(e))

        self.soc_cap_thread_active=False
        print("Send capture thread stopped!")

    def get_dropped_frames(self):  # Function to return the count of the dropped frames
        return self.frame_drop_count

if __name__ == '__main__':
    bot=FumeBotSocket()

    while True:
        if bot.is_socket_data_available():
            data=bot.get_socket_data()

            bot.put_socket_data("Hello There!")