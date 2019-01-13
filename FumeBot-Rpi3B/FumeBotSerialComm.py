#!/usr/bin/python2

"""
FUMEBOT SERIAL COMMUNICATION

This python module tries to connect with the Arduino AVR microcontroller
via UART/Serial connection. The module using threading for the receiving
and transmitting serial data.

Written by  :  Ajith Thomas
Date        :  5-4-2018
"""

import serial
import time
from queue import Queue
from threading import Thread


class FumeBotSerial(object):

    q_serial_r=Queue(maxsize=15)  # Queue to put the serial received data
    q_serial_s=Queue(maxsize=15)  # Queue to put the serial data to send

    def __init__(self,serial_port='/dev/serial0',baud=115200):
        super(FumeBotSerial, self).__init__()

        self.port=serial_port
        self.baud=baud
        self.serial_port_connected=False

        self.serial_port=serial.Serial()  # Create an object of serial

        try:  # Try to open the port
            self.serial_port=serial.Serial(self.port, self.baud, timeout=0)
            self.serial_port_connected=True
        except (OSError,serial.SerialException) as e:
            print("Initializing port error: "+str(e))
            self.serial_port_connected=False

        # Read thread
        self.r_thread=Thread(target=self.read_from_serial_port,args=(self.q_serial_r,))
        self.r_thread.setDaemon(True)
        self.read_thread_running = False

        # Send thread
        self.s_thread=Thread(target=self.send_via_serial_port,args=(self.q_serial_s,))
        self.s_thread.setDaemon(True)
        self.send_thread_running=False

    def open_serial_port(self):  # Function to try open the serial port if disconnection occurs
        while not self.serial_port_connected:
            try:
                self.serial_port = serial.Serial(self.port, self.baud, timeout=0)
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
                self.serial_port_connected=True
            except (OSError,serial.SerialException) as e:
                print("Opening serial port error: "+str(e))
                self.serial_port_connected=False
                time.sleep(1)
                continue

    def start_serial_read_thread(self):  # Function to start the read thread
        if not self.read_thread_running:
            self.read_thread_running=True
            self.r_thread.start()
            return self.read_thread_running

    def start_serial_send_thread(self):  # Function to start the send thread
        if not self.send_thread_running:
            self.send_thread_running=True
            self.s_thread.start()
            return self.send_thread_running

    def stop_serial_read_thread(self):  # Function to stop the read thread
        self.read_thread_running=False
        return self.read_thread_running

    def stop_serial_send_thread(self):  # Function to stop the send thread
        self.send_thread_running=False
        return self.send_thread_running

    def start_all_serial_threads(self):  # Function to start all the serial thread
        self.start_serial_read_thread()
        self.start_serial_send_thread()

    def stop_all_serial_threads(self):  # Function to stop all the serial thread
        self.read_thread_running=False
        self.send_thread_running=False

    def is_serial_thread_running(self):  # Function to check whether the serial is running
        if self.send_thread_running and self.read_thread_running:
            return True
        else:
            return False

    def is_serial_connected(self):  # Is the serial port connected
        return self.serial_port_connected

    def get_serial_data(self):  # Function to get received serial data from the queue
        if self.q_serial_r.qsize() > 0:
            data=self.q_serial_r.get()
            return data

    def is_serial_data_available(self):  # Function to check whether data is available in the queue
        if self.q_serial_r.qsize() > 0:
            return True
        else:
            return False

    def put_serial_data(self, data):  # Function to put serial data in the queue to be sent
        if not self.q_serial_s.full() and self.serial_port_connected:
            self.q_serial_s.put(data)

    def read_from_serial_port(self,qr):  # Function to get data from the serial (Threaded)

        data=b''
        rx_data=b''
        rx_data_str=""

        while self.read_thread_running is True:
                self.open_serial_port()  # If serial port is not available try to connect

                try:
                    if self.serial_port.in_waiting > 0:  # If the number of bytes available to read is greater than zero
                        data=self.serial_port.read(1)  # Read one byte of the data

                        if data == b'\x00': # Check for parity violations in the serial stream and if found reset
                            self.serial_port.reset_input_buffer()
                            data=b''
                            rx_data=b''
                            rx_data_str=""
                            continue  # Don't do the below steps

                        rx_data=rx_data+data  # Add the byte to previous byte

                except (OSError,serial.SerialException) as e:  # If an exception occurs clear the data
                    rx_data_str=""
                    rx_data=b''
                    data=''
                    self.serial_port_connected=False
                    self.serial_port.reset_input_buffer()  # Reset the input buffer to clear any partial data
                    self.serial_port.close()
                    print("Serial read error: "+str(e))
                    time.sleep(1)
                    continue

                if data == b'\r' or data == b'\n':  # Look for the newline or carriage
                    rx_data_str=str(rx_data)  # Convert to string format
                    rx_data_str=rx_data_str.strip()  # Strip the carriage return and newline

                    if len(rx_data_str) > 0:  # If the data length is greater than zero
                        if not qr.full():  # If the queue is not full
                            qr.put(rx_data_str)

                    rx_data=b''

        print("Serial read thread stopped!")

    def send_via_serial_port(self,qs):  # Function to send data via serial (Threaded)

        while self.send_thread_running is True:
            """
            Only one thread needs to have the serial reconnect capability, for this 
            case the read thread is the one that is started so there is no need for a
            serial reconnect on this thread
            """
            if qs.qsize() > 0:
                try:
                    self.serial_port.write(qs.get())

                except (OSError,serial.SerialException) as e:
                    self.serial_port_connected=False
                    self.serial_port.reset_output_buffer()
                    self.serial_port.close()
                    print("Serial send error: "+str(e))
                    time.sleep(1)

    def is_send_serial_free(self):  # Function to check if the output buffer is ready to write
        if self.serial_port_connected:
            try:
                if self.serial_port.out_waiting <= 0:
                    return True
                else:
                    return False

            except (OSError, serial.SerialException) as e:
                self.serial_port_connected=False
                self.serial_port.reset_output_buffer()
                self.serial_port.close()
                print("Serial send check error: "+str(e))
                time.sleep(1)

        else:
            return False


if __name__ == '__main__':

    ser = FumeBotSerial()

    ser.start_serial_read_thread()
    ser.start_serial_send_thread()

    while True:

        if ser.is_send_serial_free():
            ser.put_serial_data("Hello\n")
            time.sleep(0.01)

        if ser.is_serial_data_available():
            rx = ser.get_serial_data()
            print(rx)


