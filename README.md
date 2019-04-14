# **FumeBot**

FumeBot is a Deep Neural Network controlled robot. The mobile robot is autonomous and can
be fitted with a range of sensors for different purposes.

![FumeBot](./Images/FumeBot.jpg =640x)

## Table of Contents
- [**FumeBot**](#fumebot)
  - [Table of Contents](#table-of-contents)
  - [Motivation](#motivation)
  - [Workings](#workings)
      - [The Mobile Robot](#the-mobile-robot)
      - [The Deep Neural Network](#the-deep-neural-network)
      - [The Graphical User Interface (GUI)](#the-graphical-user-interface-gui)
  - [Testing and Observations](#testing-and-observations)
  - [Requirements and Installation](#requirements-and-installation)
      - [Hardware Requirements](#hardware-requirements)
      - [Software Requirements](#software-requirements)
  - [Usage](#usage)
      - [Connecting to FumeBot](#connecting-to-fumebot)
      - [Controlling FumeBot](#controlling-fumebot)
      - [Collecting the Training Data](#collecting-the-training-data)
      - [The Deep Neural Network (AlexNet)](#the-deep-neural-network-alexnet)
      - [Data Communication and Video Transmission](#data-communication-and-video-transmission)
  - [Issues](#issues)

## Motivation

This was done as a masters final project. The idea was to check different kinds of deep neural network architectures that could be used for autonomous navigation. At the moment the only neural network that was tested is AlexNet which is a Deep Convolutional Neural Network by [Krizhvesky *et al*](https://papers.nips.cc/paper/4824-imagenet-classification-with-deep-convolutional-neural-networks.pdf).

## Workings

#### The Mobile Robot

FumeBot is a differential drive robot with a Raspberry Pi 3B and Arduino Mega 2560 on-board. The Raspberry is connected to a Raspberry Pi 5MP camera and also a Lepton 3 thermal camera. The Arduino interfaces with the stepper motor drivers, sensors and a 3G module, the data from which are sent back to the Raspberry through a UART connection. The Raspberry connects to a laptop through WiFi where the DNN is being run for inference. The DNN is being run on the laptop because if complex models are used then more processing power is available.

![Communication](./Images/OverallCommArchitecture.jpg =640x)

The Raspberry Pi handles the heavy task of maintaining communication with the laptop, sending camera frames from both the RPi camera and Lepton back to the main computer. It also handles sending or receiving commands or data from the Main computer or Arduino microcontroller.

The robot is powered by two battery banks, one for driving the servo and stepper motors while the other is being used to power the Raspberry Pi 3, Arduino MCU, sensors and the 3G module. The range of motion available to the robot is shown below:

![Movement](./Images/FumeBotMovement.jpg =320x)

In addition to controlling the robot's movement, the camera fork mount can be controlled as well to look around. The sensors carried by the robot are a gas sensor for measuring CO2 and TVOC (Total Volatile Organic Compound), particle sensor for detecting smoke and an environmental sensor for measuring pressure, temperature and humidity. Thresholds can be used to alert the user if the robot has detected smoke or high amounts of CO2 or TVOC. The alert can be sent over WiFi or through the 3G module by SMS if WiFi is not available.

#### The Deep Neural Network

As mentioned earlier the DNN used for the robot is a Convolutional Neural Network (CNN) which is a slight variation of AlexNet and can be seen in the image below.

![CNN](./Images/AlexNetFumeBot.jpg =640x)

For training the CNN, supervised learning was used in which data was collected by manually driving the robot and recording the camera frames along with the WASD key presses associated with that frame. The CNN was then trained to classify the correct key to press based on the frame received from the camera. The camera image given to the CNN was a grayscale 80x60 and the output from it is a one hot vector for example, `[0, 1, 0]` which is `[Forward left turn, Forward, Forward right turn]` of this `Forward` is selected as it is set to one.

The full range of motion available to the robot was not used to test whether the neural network could actually perform a simple task like navigating around an obstacle course. As a result only forward, forward left and forward right motion of the robot was used.

#### The Graphical User Interface (GUI)

A GUI was made to control the robot and to change the settings of the robot. The main function of the GUI is to display the image coming from the cameras, control the robot's movement and also the camera mount, record training data like the camera image and the key strokes as a one hot vector, video recording, changing the sensors thresholds and connection settings. The following is the GUI and the video stream coming from the robot displayed in the GUI respectively: 

![GUI](./Images/FumeBotGUI.png =640x)

![GUI video](./Images/LabTestRobot.gif =640x)

## Testing and Observations

The AlexNet CNN was trained on data collected from a simple obstacle course and was tested on the same and other obstacle courses to see if
it has generalized well. The obstacles when collecting the training data were orange cylinders but during testing other colors and different arrangement of
the obstacle was used to test whether it would confuse the neural network.

![FumeBot Lab Test Video](./Images/LabTestHandheld.gif)

On testing the robot with different obstacle courses it was found that the robot has learned to navigate the courses. The obstacle color did not confuse the robot probably because grayscale images were used for training and inference which helped in not overfitting over the color of the obstacle used. But human biases
were learned by the network, for example, in order to make it obvious to the network to avoid the obstacle, it was intentionally driven up to the obstacle and avoided at the last moment which caused the neural network to follow the obstacle if moved and then avoid it. The robot was able to stay in the obstacle 
course and complete it but did make mistakes and collided with the obstacle. This is caused by not using a wide angle lens and using the standard RPi camera lens which has a limited field of view.

## Requirements and Installation

This section describes the the hardware and software requirements for FumeBot. The robot is designed in Autodesk Inventor 2018. The parts and assembly files are available [here](./FumeBot-Design/).

#### Hardware Requirements
The schematics for wiring up FumeBot can be found [here](./FumeBot-Circuit_Design/Fumebot%20Circuit.pdf).
- Raspberry Pi 3B
- Raspberry Pi Camera v1.3 (5MP)
- [FLIR Lepton 3 Camera](https://groupgets.com/manufacturers/flir/products/lepton-3-0) 
- [Adafruit FONA 3G Cellular + GPS Breakout (SIMCom SIM5320A)](https://learn.adafruit.com/adafruit-fona-3g-cellular-gps-breakout/downloads)
- Arduino MEGA 2560
- [Adafruit BME280 (Environmental Sensor)](https://www.adafruit.com/product/2652)
- [SparkFun CCS811 (Gas Sensor)](https://www.sparkfun.com/products/14193)
- [SparkFun MAX30105 (Particle Sensor)](https://www.sparkfun.com/products/14045)
- [DRV8834 Stepper Motor Driver](https://www.pololu.com/product/2134)
- [Pololu Boost Regulator](https://www.pololu.com/product/799)
- [Stepper Motors](https://www.omc-stepperonline.com/nema-17-bipolar-18deg-44ncm-623ozin-085a-53v-42x42x48mm-4-wires-17hs19-0854s.html)
- [TowerPro SG90 Servo Motors](http://www.ee.ic.ac.uk/pcheung/teaching/DE1_EE/stores/sg90_datasheet.pdf)
- BC547 Transistors
- RGB LED (Used as a status LED)

#### Software Requirements
The following software needs to be installed before running the main programs.

- For Windows PC (The GUI application was not tested on Linux)
  - OpenCV 3.4.3
  - Numpy 1.14.5
  - TensorFlow 1.10.0
  - Keras 2.2.2
  - Pandas 0.23.4 
  - PyQt4 4.11.3
  
  The above packages are for Python 3.6. Both OpenCV and PyQt4 can be installed using pip from [here](https://www.lfd.uci.edu/~gohlke/pythonlibs/). The rest can be installed by just using pip.

- For Raspberry Pi 3B (The RPi 3B application needs to be run with python 2.7)
  - PySerial 3.4
  - OpenCV 3.4.1
  - Numpy 1.14.2
  - RPI.GPIO
  
  The above packages are for Python 2.7. OpenCV needs to compiled from source and installed using instructions 
  found [here](https://www.pyimagesearch.com/2015/07/27/installing-opencv-3-0-for-both-python-2-7-and-python-3-on-your-raspberry-pi-2/). OpenCV was installed without Python Virtual Environment.

  The `FumeBot-RPi3B` file needs to be copied to `/home/pi/` directory of the Raspberry Pi and the `botsetup.sh` script should be run. This will create a service that starts every time the Raspberry boots and can be controlled using systemctl.

  ```bash
  # To run the script
  sudo chmod +x botsetup.sh
  ./botsetup.sh

  # To enable or disable the FumeBot service
  sudo systemctl enable fumebot.service
  sudo systemctl disable fumebot.service

  # To start or stop the FumeBot service
  sudo systemctl start fumebot.service
  sudo systemctl stop fumebot.service
  ```

- For Arduino MEGA 2560 
  - Adafruit BME280 Library
  - Adafruit FONA Library
  - SparkFun CCS811 Library
  - SparkFun MAX3010x Pulse and Proximity Sensor Library
  
  The FumeBOT.ino file must be flashed to the Arduino MEGA.

## Usage

This section describes how to use the software and to control the robot. Additional information about source code is also provided.

#### Connecting to FumeBot

The FumeBot GUI application needs to be started first by running `python FumeBot.py` on the laptop or main computer, then the connection setting needs to be changed to IP of the host computer (Laptop) to open the ports and start listening. In Window, the IP address of the machine can be found out by running `ipconfig` in command prompt and looking at the IPv4 Address section. The GUI running on the PC is the server and the Raspberry is the client, both the machines needs to have the same IP and ports for them to be able to connect. The IP and ports configuration of Fumebot can be changed in the `FumeBot-Rpi3B/bot_config.ini` file. The FumeBot service needs to be running as well.

#### Controlling FumeBot

The WASD keys can be used to control the movement of the robot and the mouse can be used to control the camera mount to tilt and pan. The mouse needs to be dragged across the video display area while holding down the left mouse button. The video capture settings can be changed to an appropriate resolution so that video lag can be reduced.

#### Collecting the Training Data

The training data recording can be started or stopped by pressing 'T' key or the "Start Recording" button can be used in the training data section. The training data is collected in the format of an image and the corresponding key pressed on receiving that image. The training data is saved as `.npy` file with an associated `.meta` file that stores the disabled keys and the order of enabled ones. For an example of reading the `.npy` file, `ViewTrainingData.py` could be run. Preprocessing maybe needed to get the data ready for training the DNN and an example of this can found in `DataSetPreprocessing.py`.

#### The Deep Neural Network (AlexNet)

The AlexNet CNN model is defined in `DeepNeuralNetworkModel.py` the methods of which are loaded into `FumeBotDNN.py`. The model is loaded and weights are applied in the constructor of the `FumeBotDNN` class. The `dnn_prediction_function` is used for inference and the output of this function is the required key the CNN wants to send to the robot. The key classification should be a one-hot vector, for example, `[1, 0, 1]`. The predicted key press vector is emitted as a signal in PyQt to the main GUI application and is handled by the `handle_dnn_key_ press` function.

For inference on images coming from the robot, the DNN must be first initialized by pressing the 'I' key or "Initialize" button. To start the prediction the 'N' key can be used to toggle it or "Activate" button can be pressed. The `FumeBotDNN.py` is loaded into the main GUI application `FumeBot.py` in which the `dnn_prediction_function` mentioned earlier is run inside the `update_video_feed_display` function.

#### Data Communication and Video Transmission

The data communication and reception of video stream is done in `FumeBotSockComm.py`. The handling of data is done in the `handle_socket_data` function and the image frames from the camera is handled in the `get_socket_stream_frames` function in the main GUI. The data is exchanged between the `FumeBotSockComm.py` and `FumeBot.py` in the form of PyQt signals as mentioned above in the case of the DNN.

The frames coming from the camera are in JPEG format and is then rendered in the GUI video frame. The figure shown below shows how the video is sent from the client (Robot) to the Server (Fumebot GUI application). Both the RPi camera and the Lepton thermal camera follows the same procedure for transmission of video.

![VideoTx](./Images/FrameSendingReceiving.jpg =640x) 

If other data needs to be sent from the robot, the `handle_socket_data` function needs to be modified. All the data communication from the Raspberry and Window PC needs to be in the following format:
```
$<CMD>,<DATA1>,<DATA2>,…,<DATA(n-1)>,#
```
Data communication between the Raspberry and Arduino is in the following format:

```
<CMD>,<DATA1>,<DATA2>,…,<DATA(n-1)>,\n
```
All the data are sent and received in plain string format.

## Issues

The following are some of the know issues observed during testing.

- The `FumeBotMain.py` which is ran on the client side can sometimes crash with an error caused by OpenCV.
- Python 2.7 is used in the client side because OpenCV could not be installed for Python 3.6. Later this was resolved but not tested. 
- The FLIR Lepton 3 thermal camera module freezes after a few minutes of usage. 
- Data built up in the client side causes a lag in getting the real time sensor measurements displayed on the GUI.
- The initialization of the DNN is slow.
- Saving the training data takes a long time if the data held in the list is large.
