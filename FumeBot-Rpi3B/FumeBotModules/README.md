## **Modules for Mobile Robot Side**

#### Communication modules for the Robot
The communication between the robot and GUI application is achieved by using the `FumeBotSocketComm.py` which uses TCP sockets for streaming data. On the other hand the communication between the Raspberry Pi and the Arduino microcontroller is realized through `FumeBotSerialComm.py` module. The serial communication is through a standard UART port available on both the Arduino MEGA and Raspberry Pi. 

The default UART available on Raspberry Pi 3B GPIO is the Mini UART which is inferior compared to the UART used for the Bluetooth modem. Therefore, the UARTs are switched so that the better one is available at the GPIO. More information on how to do this is available by following this [link](https://www.raspberrypi.org/documentation/configuration/uart.md).

#### The FLIR Lepton 3 Module
To communicate with the Lepton 3 thermal camera the `Lepton3capture.c` was slightly modified from the original available [here](https://github.com/lukevanhorn/Lepton3/blob/master/capture/capture.c). Since the main program for the robot is written in python the `Lepton3capture.c` file is compiled into shared object file and is then loaded into `Lepton3CapLib.py` python module which breaks out the necessary functions available in the original C file. The `Lepton3CapLib.py` file is then imported by `FumeBotSocketComm.py` to get the frames from the thermal camera in a thread and send it to the GUI application.