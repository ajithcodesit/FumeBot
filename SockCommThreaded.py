"""
FUMEBOT SOCKET SERVER

This is the Fumebot socket server and is used to handle both image as well
as byte string data sent over TCP/IP sockets. One class is used to handle both
type of data. In the image type mode, sending image data to the remote robot is
not necessary and therefore is not implemented.

Written by  :  Ajith Thomas
Date        :  5-4-2018
"""

import socket
import time
from threading import Thread
from PyQt4 import QtCore


class SockComm(QtCore.QObject):
    # Type of data that need to be read
    DATA_TYPE=0
    IMG_TYPE=1

    frameReceived = QtCore.pyqtSignal(bytes, bytes)  # Signal for new image frame and thermal image frame
    capStatus = QtCore.pyqtSignal(str)  # Signal for the capture status

    dataReceived = QtCore.pyqtSignal(bytes)  # Signal for new data in list form
    sockConnected = QtCore.pyqtSignal(str)  # Signal for when connection is established

    def __init__(self, host = 'localhost', port = 5000, type = DATA_TYPE):
        super(SockComm, self).__init__()

        self.host=host
        self.port=port

        self.run_type = self.read_sock_data  # default function is data read method

        if type == self.IMG_TYPE:
            self.run_type = self.read_sock_img
        else:
            self.run_type = self.read_sock_data

        self.sock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Reuse the address
        self.sock.setblocking(False)  # Blocking is set to false

        self.connected=False  # Variable to say connection has been established
        self.conn=self.sock
        self.addr=self.sock

        self.bytes=b''  # Bytes read from the socket
        self.string_bytes= b''  # String read from the

        # Bytes of the image both thermal and normal
        self.jpg_bytes=b''
        self.therm_jpg_bytes=b''

        self.sock_thread=Thread(target=self.read_from_sock_thread,args=(self.run_type,))  # The run mode is also passed
        self.sock_thread.setDaemon(True)  # Thread will close when the main program closes
        self.sock_thread_run=False  # Variable to say whether to start the thread or not

    def socket_bind(self,host,port):  # Function to bind the host and the port
        print("Binding socket...")

        try:
            print("Trying to bind: Host= "+str(host)+" Port= "+str(port))
            self.sock.bind((host,port))  # Binding completed
            print("Binding complete!")

            print("Listening...")
            self.sock.listen(10)  # Start listening up to 10 bad connection

        except socket.error as e:
            print("Binding Error: "+str(e))

    def socket_accept(self):  # Function to accept connections

        while self.sock_thread_run is True:  # Run until a connection is accepted non-blocking
            try:
                self.conn,self.addr=self.sock.accept()  # Accept the connection

                if self.addr[0] != '':
                    self.connected=True  # We are now connected set to true
                    print("Connection established! IP: " + str(self.addr[0]) + " Port: " + str(self.addr[1]))
                    return self.conn

            except socket.error as e:
                if e.errno == socket.errno.EWOULDBLOCK or e.errno == socket.errno.EAGAIN:  # Non critical error
                    time.sleep(1)  # Wait for some time 1 second
                    continue
                else:  # Critical error
                    print("Accepting error: "+str(e))
                    self.sock_thread_run = False
                    break

    def socket_close(self):  # Function to close the socket
        if self.sock_thread_run is True:
            self.stop_sock_read_thread()
            print("Socket thread stopped!")

        self.conn.close()
        self.connected=False
        # A signal should be emitted to tell the connection was closed (Done in read thread)
        print("Connection Closed!")

    def socket_shutdown(self):  # Function to shutdown the sockets before closing
        try:
            self.conn.shutdown(socket.SHUT_RDWR)
        except socket.error as e:
            print("Socket shutdown error: "+str(e))

    def start_sock_read_thread(self):  # Function to start the thread
        try:
            if not self.sock_thread_run:
                self.sock_thread_run=True  # Now the thread is running
                self.sock_thread.start()  # Start the socket thread

        except RuntimeError as e:
            print("Thread error: "+str(e))

    def stop_sock_read_thread(self):  # Function to stop the thread
        self.sock_thread_run=False
        return not self.sock_thread_run

    def read_from_sock_thread(self,run_method):  # This is executed in a different threads
        run_method()  # The function passed to this function is run

    def read_sock_img(self):  # Function to capture image streams
        print("[FRAME]Socket read thread started")

        self.socket_bind(self.host, self.port)  # Bind host and port
        self.socket_accept()  # Accept any connection in non blocking mode

        if self.connected is True:
            print("[FRAME]Connection accepted!")
            self.capStatus.emit("SAO")  # Stream active and open
        else:
            print("[FRAME]Connection failed!")

        self.sock.setblocking(False)  # Operate in non blocking mode

        while self.sock_thread_run is True:  # This is the main read loop

            try:
                self.bytes += self.conn.recv(4096)  # Read from the socket up to 1024 bytes at a time
                self.conn.send(b'\x00')  # Send a null byte to check if connection is still alive

            except socket.error as e:
                if e.errno == socket.errno.EWOULDBLOCK or e.errno == socket.errno.EAGAIN:  # Non critical error
                    continue
                elif e.errno == socket.errno.ECONNRESET or e.errno == socket.errno.ECONNABORTED:
                    self.capStatus.emit("SNA")  # Stream not available "Disconnection"
                    print("[FRAME]Socket read error: " + str(e))
                    break
                else:
                    print("[FRAME]Socket read unhandled error: " + str(e))
                    break

            # Modified byte string are used to find the marker for the jpeg stream
            start_marker=self.bytes.find(b'\RS\xff\xd8')  # Marker for the start of jpg frame
            end_marker=self.bytes.find(b'\xff\xd9\RE')  # Marker for the end of jpg frame

            # Modified bytes string are used to find the start and end of the thermal frame
            start_marker_therm=self.bytes.find(b'\TS\xff\xd8')
            end_marker_therm=self.bytes.find(b'\xff\xd9\TE')

            if start_marker_therm != -1 and end_marker_therm != -1:  # This is for the thermal image
                self.therm_jpg_bytes = self.bytes[start_marker_therm + 3:end_marker_therm + 2]
                self.bytes = self.bytes[end_marker_therm + 2+3:]

            # The main camera image is used as the signal driver here to also push thermal images
            if start_marker != -1 and end_marker != -1:  # If the marker is not found the returned values is -1
                self.jpg_bytes = self.bytes[start_marker + 3:end_marker + 2]  # The jpeg bytes in string form
                self.bytes = self.bytes[end_marker + 2+3:]  # The extra bytes of the next frame is stored as well

                self.frameReceived.emit(self.jpg_bytes, self.therm_jpg_bytes)  # Send the jpeg bytes

        print("[FRAME]Thread exited!")
        self.socket_close()  # Closing the thread

    def read_sock_data(self):  # Function to capture data streams
        print("[DATA]Socket read thread started")

        self.socket_bind(self.host, self.port)  # Bind host and port
        self.socket_accept()  # Accept any connection in non blocking mode

        if self.connected is True:
            print("[DATA]Connection accepted!")
            self.sockConnected.emit("DAO")  # Data read active and open
        else:
            print("[DATA]Connection failed!")

        self.sock.setblocking(False)  # Operate in non blocking mode

        while self.sock_thread_run is True:

            try:
                self.string_bytes += self.conn.recv(64)  # Read from the socket up to 64 bytes at a time string format
                self.conn.send(b'\x00')  # Send a null byte to check if connection is still alive

                start_marker=self.string_bytes.find(b'$')
                end_marker=self.string_bytes.find(b'#')

                if start_marker != -1 and end_marker != -1:
                    data_bytes=self.string_bytes[start_marker+1:end_marker]  # The start and end markers are stripped
                    self.string_bytes=self.string_bytes[end_marker+1:]  # Bytes set to after the end marker

                    self.dataReceived.emit(data_bytes)  # Cast as string and send it as signal

                    # Decoding of the byte string is not done here

            except socket.error as e:
                if e.errno == socket.errno.EWOULDBLOCK or e.errno == socket.errno.EAGAIN:  # Non critical error
                    continue
                elif e.errno == socket.errno.ECONNRESET or e.errno == socket.errno.ECONNABORTED:
                    self.sockConnected.emit("DNA")  # Data not available "Disconnection"
                    print("[DATA]Socket read error: " + str(e))
                    break
                else:
                    print("[DATA]Socket read unhandled error: " + str(e))
                    break

        print("[DATA]Thread exited")
        self.socket_close()  # Closing the connection

    def send_via_socket(self,data_str):  # Function to send the data

        self.conn.setblocking(True)  # Set as blocking for a few moments

        try:
            byte_str=data_str.encode('utf-8')
            self.conn.sendall(byte_str)  # Send all the data at once
        except socket.error as e:
            print("Socket send error: "+str(e))
            self.socket_close()

        self.conn.setblocking(False)  # Set as non-blocking again


if __name__ == '__main__':
    soc=SockComm('192.168.137.1', 8089, SockComm.IMG_TYPE)
    soc.start_sock_read_thread()

    # FOR TESTING ONLY!
    time.sleep(1000000)  # If main stop, thread stops as daemon set to true






