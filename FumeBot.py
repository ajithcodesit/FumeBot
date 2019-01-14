
"""
FUMEBOT GUI APPLICATION

This is GUI application to connect and communicate with FumeBot, send and receive data
over TCP/IP sockets to control the robot. This application uses several threads for socket
communication

Written by  :  Ajith Thomas
Date        :  5-4-2018
"""

import os
import sys
import cv2
import time
import datetime
import ctypes
import numpy as np
from PyQt4 import QtCore, QtGui
from FumeBot_UI import Ui_mainWindow
from FumeBotDataSaver import FumeBotVideoSaver, FumeBotTrainingDataSaver
# from FumeBotDNN import FumeBotDNN
from SockCommThreaded import SockComm
from configparser import ConfigParser


class MainWindow(QtGui.QMainWindow):

    # Configuration file
    config_path="config.ini"

    # Application version
    app_version="v0.1"
    app_name="FumeBot"
    app_id=app_name+"."+app_version

    # Type of messages
    app_msg="Interface"
    bot_msg="Robot"

    # Socket connection variables
    socket_data_connected=False  # For data stream
    socket_img_connected=False   # For image stream

    # Controller status
    micro_controller_rdy=False

    prev_msg_disp_time=0  # Variable stores the last time the data not connected port message
    msg_delay=2  # The delay for it (s)

    # Camera display and processing
    frame_black=np.zeros((720,1280,3),dtype=np.uint8)
    frame_bgr=np.zeros((720,1280,3),dtype=np.uint8)
    frame_train_bgr=np.zeros((720, 1280, 3), dtype=np.uint8)
    frame_dnn_bgr=np.zeros((60,80,3), dtype=np.uint8)
    frame_thermal=np.zeros((720,1280,3),dtype=np.uint8)
    frame_thermal_copy=np.zeros((120,160,3),dtype=np.uint8)
    final_frame=np.zeros((720, 1280, 3), dtype=np.uint8)

    display_fps=30  # FPS for the video feed display (This affects only the UI display and not the actual camera FPS)

    cam_actual_fps=0
    prev_cam_rcv_time=time.time()
    therm_actual_fps=0
    prev_therm_rcv_time=time.time()

    normal_cam_enable=True
    thermal_cam_enable=False

    # Variables for blending
    norm_cam_weight=50.0
    therm_cam_weight=50.0

    # Variables for adjusting the thermal view
    therm_scale_val=1
    therm_pos_horz=0
    therm_pos_vert=0

    # Gas sensor readings
    eCO2=0
    TVOC=0

    # Particle sensor readings
    red=0
    green=0
    ir=0

    # Environmental sensor readings
    pressure=0
    temperature=0
    humidity=0

    # Gas sensor thresholds
    eCO2_thresh=0
    TVOC_thresh=0
    gas_threshold_reached=False  # To say whether the threshold has been reached or not

    # Particle sensor threshold
    Red_thresh=0
    Green_thresh=0
    IR_thresh=0
    particle_threshold_reached=False  # To say whether the threshold has been reached or not

    flash_warning=False  # Boolean to say when to flash the warning
    flash_non_critical=False  # Boolean to say when to flash the non critical warnings
    flash_time_w=250  # Flash timer interval for warnings
    flash_timer_nc=500  # Flash timer interval for non critical warnings

    # Max values for gas sensor
    cur_max_eCO2=0
    cur_max_TVOC=0

    # Max values for particle sensor
    cur_max_Red=0
    cur_max_Green=0
    cur_max_IR=0

    # Neural net
    nn_active=False

    # SMS alert variable
    sms_number="09876543210"
    sms_enabled=0

    # For the WASD key control
    button_repeat_delay=25  # Movement button delay
    key_pressed_dict={
        'W' : False,
        'A' : False,
        'S' : False,
        'D' : False,
        'SHIFT' : False,
        'CTRL' : False
    }

    default_key_pressed_dict=key_pressed_dict.copy()  # Make a shallow copy

    move_button_press_order_dict = {  # 10 Control options
        'FORWARD': 0,
        'BACKWARD': 1,
        'LEFT': 2,
        'RIGHT': 3,
        'LEFT_TURN_FORWARD': 4,
        'RIGHT_TURN_FORWARD': 5,
        'LEFT_TURN_BACKWARD': 6,
        'RIGHT_TURN_BACKWARD': 7,
        'ACCELERATION': 8,  # Should not be used in training
        'BRAKE': 9,  # Should not be used in training
    }

    # This a dictionary for the DNN button press control
    dnn_move_button_press_order_dict = {  # 10 Control options
        'FORWARD': 1,
        'BACKWARD': 0,
        'LEFT': 0,
        'RIGHT': 0,
        'LEFT_TURN_FORWARD': 0,
        'RIGHT_TURN_FORWARD': 2,
        'LEFT_TURN_BACKWARD': 0,
        'RIGHT_TURN_BACKWARD': 0,
        'ACCELERATION': 0,  # Should not be used in training
        'BRAKE': 0,  # Should not be used in training
    }

    default_move_list=[]  # This list is dynamically made by checking the keys that are allowed to be pressed
    move_button_press_list=[]  # The list for the button pressed

    # Can also be used to not send the actual command to the robot as well
    # This kind of disabling method was used for finer control and can be divided into 3 sections and disabled if needed
    disabled_button_press_dict={  # These button/combination of button won't be saved in the training data
        'FORWARD': False,   # Basic controls
        'BACKWARD': False,
        'LEFT': False,
        'RIGHT': False,
        'LEFT_TURN_FORWARD': False,  # Combinational controls
        'RIGHT_TURN_FORWARD': False,
        'LEFT_TURN_BACKWARD': False,
        'RIGHT_TURN_BACKWARD': False,
        'ACCELERATION': False,  # Additional controls
        'BRAKE': False,  # False means not disabled and True means disabled
    }

    # This dictionary is used for the button press control of the DNN
    dnn_disabled_button_press_dict = {  # These button/combination of button won't be saved in the training data
        'FORWARD': False,  # Basic controls
        'BACKWARD': True,
        'LEFT': True,
        'RIGHT': True,
        'LEFT_TURN_FORWARD': False,  # Combinational controls
        'RIGHT_TURN_FORWARD': False,
        'LEFT_TURN_BACKWARD': True,
        'RIGHT_TURN_BACKWARD': True,
        'ACCELERATION': True,  # Additional controls
        'BRAKE': True,  # False means not disabled and True means disabled
    }

    # Copies for the list and dictionaries that are updated according to user changes (Used in UI update)
    updated_key_order_dict=move_button_press_order_dict.copy()  # Shallow copy is made
    updated_disabled_key_dict=disabled_button_press_dict.copy()  # Shallow copy is made

    # The total number of allowed key combination is calculated using the disabled button dictionary (Updated)
    keys_comb_len=len(disabled_button_press_dict)  # Number of different key presses

    # According to the disabled buttons dictionary the commands for that movement won't be send to the robot
    disable_send_command=False  # False means not disabled and True means disabled

    disabled_move_key_pressed=False  # Variable to say whether a disabled key was pressed or not (True means pressed)

    # Variables for the training data generation
    train_file_name='training_dataset.npy'
    train_file_path='~\\Documents\\FumeBot\\Training'
    train_path_exists=False
    train_frame_width=80
    train_frame_height=60
    training_data=[]  # This list contains the image and control input (Key presses)
    save_per_every=250  # After 250 training samples a save is done
    enable_training_data_recording=False
    training_data_recording_paused=False
    save_done=False
    training_frame_count=0

    # Variables for the training numpy meta file
    parent_file_name=train_file_name
    meta_key_order={}
    meta_disabled_key={}
    ko_comp_failed = False
    dk_comp_failed = False

    # For the mouse button control
    mouse_pressed_dict={
        'LEFT_MB':False,
        'MID_MB':False,
        'RIGHT_MB':False,
    }

    # Used for nulling
    start_x=0
    start_y=0

    # These are the component x and y values for the mouse position after nulling
    mouse_x=0
    mouse_y=0

    pan_scaler=4.0  # This is amount of mouse movement needed (Higher the scaler the more mouse movement is needed)
    tilt_scaler=4.0

    max_pan_tilt_scaler=10.0

    pan_tilt_limit_dict={
        'TILT_UP':180,
        'TILT_DOWN':0,
        'PAN_LEFT':0,
        'PAN_RIGHT':180,
    }

    dpp_tilt=(180.0/float(pan_tilt_limit_dict['TILT_UP']-pan_tilt_limit_dict['TILT_DOWN']))/tilt_scaler
    dpp_pan=(180.0/float(pan_tilt_limit_dict['PAN_RIGHT']-pan_tilt_limit_dict['PAN_LEFT']))/pan_scaler

    pan_disp=0  # This can be shown on the display
    tilt_disp=0  # This can be shown on the display

    pan_angle=0  # The final pan angle
    tilt_angle=0  # The final tilt angle

    pan_default=90
    tilt_default=90

    enable_mouse_pan_tilt=True  # False means disabled

    # Commands for movement
    movement_cmd_dict={
        'FRWD' : 109,
        'BWRD' : 113,
        'LFT'  : 127,
        'RGT'  : 179,
        'L_TURN_F' : 181,
        'R_TURN_F' : 191,
        'L_TURN_B' : 193,
        'R_TURN_B' : 197
    }

    brake=0  # Brake not state (0-Not applied, 1-Applied)
    accl=0  # Increase speed state (0-Not used, 1-Used)

    # Contains the header for the socket receives (This dictionary is the send on the other side)
    socket_receive_header_dict = {
        'GAS': 'G',
        'ENVIRONMENTAL': 'E',
        'PARTICLE': 'P',
        'WIFI': 'W',
        'SOCK_SET_ACK': 'AK',  # This is used as an acknowledgement saying settings where applied successfully
        'GAS_THR_RET': 'GTR',
        'PAR_THR_RET': 'PTR',
        'SMS_RET': 'SR',
        'CAM_RET': 'CMR',
        'MCU_STAT': 'MS',
    }

    ack_code_dict = {
        'GAS_THR': 1,
        'PAR_THR': 2,
        'SMS_SET': 3,
        'CON_SET': 4,
        'VID_SET': 5,
        'RST_ALM': 6,
    }

    # Contains the header for the socket sending (This dictionary is the receive on the other side)
    socket_send_header_dict={
        'MOVE': 'MV',  # For movements of the robot
        'CAM_PT': 'CPT',  # Camera pan and tilt
        'CAM_SET': 'CM',
        'COM_SET': 'CO',
        'GAS_THR': 'GT',
        'PAR_THR': 'PT',
        'SMS': 'SM',
        'REBOOT_RPI': 'RRP',
        'RESET_UC': 'RUC',
        'PWR_OFF_RPI': 'PWR',
        'RESET_ALARM': 'RA',  # Reset the alarm state in the microcontroller
        'RETRIEVE': 'RET',  # This is used to retrieve settings from the RPi side (The headers will be the parameter)
    }

    # Contains the settings that got updated after retrieving the information from the RPi side
    updated_settings_dict={
        'CAM_SET': False,
        'GAS_THR': False,
        'PAR_THR': False,
        'SMS': False,
    }

    # Wifi info
    wifi_link_quality=0
    wifi_signal_level=0

    # Video HUD
    enable_HUD=1  # 1 means enabled, 0 means enabled

    # Recording video
    video_file_name='Video.avi'
    video_file_path='~\\Documents\\FumeBot\\Video'
    video_path_exists=False
    rec_width=1280
    rec_height=720
    rec_res_index=1
    rec_fps=20
    enable_video_recording=False
    video_recording_paused=False
    use_cap_resolution=0 # Use the video feed capture resolution
    use_cap_fps=0  # Use the video feed capture FPS

    def __init__(self,parent=None):
        super(MainWindow,self).__init__(parent)

        self.config=ConfigParser()

        try:  # Try to read the configuration file
            self.config.read_file(open(self.config_path))
        except FileNotFoundError:  # Make a new configuration file
            self.make_config_file()

        self.config.read(self.config_path)

        self.host='192.168.137.1'  # Network IP

        self.port_img=8089  # Port for receiving image frames from
        self.port_data=8090  # Port for receiving data

        # This shows the icon of the app instead of the python program icon (Only needed for window)
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(self.app_id)  # Will not work in Linux

        # Objects of the UI
        self.ui=Ui_mainWindow()
        self.ui.setupUi(self)

        # TCP socket stream image
        self.soc_img=SockComm(self.host, self.port_img, SockComm.IMG_TYPE)

        # TCP socket stream data
        self.soc_data=SockComm(self.host, self.port_data, SockComm.DATA_TYPE)

        self.display_info(self.app_msg,self.app_name+" "+self.app_version)  # Display the app name and version

        # self.dnn=FumeBotDNN()  # Load the Neural Network

        self.connection_to_signals()
        self.attributes_of_ui()

        self.update_defaults_from_config()  # Update the UI from the configuration file

        # Video saver for the bot
        self.vid_saver = FumeBotVideoSaver(name=self.video_file_name, path=self.video_file_path,
                                           width=self.rec_width, height=self.rec_height, fps=self.rec_fps)

        # Training data saver for the bot
        self.train_data_saver = FumeBotTrainingDataSaver(name=self.train_file_name, path=self.train_file_path,
                                                         saves_ps=self.save_per_every)

        self.check_file_path_statuses()  # Check the video and training data save path

    def connection_to_signals(self):  # The signal from the UI and other signals are connected here

        # For the mouse movement over the feed display widget
        self.ui.video_frame.setMouseTracking(True)
        self.ui.video_frame.installEventFilter(self)

        # Signal for connecting button
        self.ui.connectButton.clicked.connect(self.establish_connection)

        # Signal for the new frame received from the MJPEG video stream
        self.soc_img.frameReceived.connect(self.get_socket_stream_frames)
        self.soc_img.capStatus.connect(self.capture_status_action)

        # Signals for the new data received
        self.soc_data.dataReceived.connect(self.get_socket_stream_data)
        self.soc_data.sockConnected.connect(self.data_status_action)

        # This is the timer that calls the handle button press function when WASD keys are pressed
        self.button_handle_timer = QtCore.QTimer()
        self.button_handle_timer.timeout.connect(self.handle_key_presses)
        self.button_handle_timer.setInterval(self.button_repeat_delay)  # Set the delay for the repeat

        # This is the timer that is used to refresh the display panel
        self.display_update_timer=QtCore.QTimer()
        self.display_update_timer.timeout.connect(self.update_video_feed_display)
        self.display_update_timer.setInterval(self.calc_interval_from_FPS())
        self.display_update_timer.start()

        # Alarm reset button
        self.ui.alarmResetButton.clicked.connect(self.reset_alarm)

        # Set all thresholds button
        self.ui.setAllThreshButton.clicked.connect(self.set_all_thresholds)

        # Get all configuration button
        self.ui.retreiveAllConfigButton.clicked.connect(self.retrieve_configurations)

        # Video recording start and stop buttons
        self.ui.startVideoRecButton.clicked.connect(self.start_vid_recording_button_clicked)
        self.ui.stopVideoRecButton.clicked.connect(self.stop_vid_recording_button_clicked)

        # Training data recording start and stop buttons
        self.ui.startTrainRecButton.clicked.connect(self.start_training_data_rec_button_clicked)
        self.ui.stopTrainRecButton.clicked.connect(self.stop_training_data_rec_button_clicked)

        # Connections for signals from the set and reset button from the connections toolbox
        self.ui.connectSaveButton.clicked.connect(self.connection_save_clicked)
        self.ui.connectResetButton.clicked.connect(self.connection_reset_clicked)

        # Connection for signals from video feed settings toolbox
        self.ui.videoSetButton.clicked.connect(self.video_feed_config_set_clicked)
        self.ui.videoResetButton.clicked.connect(self.video_feed_config_reset_clicked)

        self.ui.enableThermalCheckBox.stateChanged.connect(self.enable_thermal_video_checkbox_changed)

        # Connections for the video display settings toolbox
        self.ui.feedTypeComboBox.currentIndexChanged.connect(self.display_mode)
        self.ui.blendSlider.sliderMoved.connect(self.blend_slider)

        self.ui.thermBlendScalerSlider.valueChanged.connect(self.thermal_image_scale_slider)
        self.ui.thermPosHorzSlider.valueChanged.connect(self.thermal_image_pos_horz_slider)
        self.ui.thermPosVertSlider.valueChanged.connect(self.thermal_image_pos_vert_slider)

        self.ui.feedSetButton.clicked.connect(self.video_disp_set_clicked)
        self.ui.feedResetButton.clicked.connect(self.video_disp_reset_clicked)

        self.ui.enableHUDCheckBox.stateChanged.connect(self.HUD_checkbox_changed)

        # Connections for the video recording settings toolbox
        self.ui.videoRecSetButton.clicked.connect(self.video_recording_config_set_clicked)
        self.ui.videoRecResetButton.clicked.connect(self.video_recording_config_reset_clicked)

        self.ui.VideoFileOpenButton.clicked.connect(self.open_video_recording_file_location)
        self.ui.videoPathBrowseButton.clicked.connect(self.browse_for_video_recording_file_path)

        self.ui.useCaptureResCheckBox.stateChanged.connect(self.disable_rec_vid_res_fps)
        self.ui.useCaptureFPSCheckBox.stateChanged.connect(self.disable_rec_vid_res_fps)

        # Connections for the training data recording settings toolbox
        self.ui.trainingSetButton.clicked.connect(self.training_recording_config_set_clicked)
        self.ui.trainingResetButton.clicked.connect(self.training_recording_config_reset_clicked)

        self.ui.trainingFileOpenButton.clicked.connect(self.open_training_data_file_location_and_select)
        self.ui.trainingFilePathBrowseButton.clicked.connect(self.browse_for_training_recording_file_path)

        # Connection for the control key disabling and order changing section
        self.ui.disableKeysSetButton.clicked.connect(self.disabled_key_config_set_clicked)
        self.ui.disableKeysResetButton.clicked.connect(self.disabled_key_config_reset_clicked)

        self.ui.forwardCheckBox.stateChanged.connect(lambda : self.disable_key_order_spin_box(redistribute=True))
        self.ui.backwardCheckBox.stateChanged.connect(lambda : self.disable_key_order_spin_box(redistribute=True))
        self.ui.leftCheckBox.stateChanged.connect(lambda : self.disable_key_order_spin_box(redistribute=True))
        self.ui.rightCheckBox.stateChanged.connect(lambda : self.disable_key_order_spin_box(redistribute=True))
        self.ui.leftTurnForwardCheckBox.stateChanged.connect(lambda : self.disable_key_order_spin_box(redistribute=True))
        self.ui.rightTurnForwardCheckBox.stateChanged.connect(lambda : self.disable_key_order_spin_box(redistribute=True))
        self.ui.leftTurnBackwardCheckBox.stateChanged.connect(lambda : self.disable_key_order_spin_box(redistribute=True))
        self.ui.rightTurnBackwardCheckBox.stateChanged.connect(lambda : self.disable_key_order_spin_box(redistribute=True))
        self.ui.accelerationCheckBox.stateChanged.connect(lambda : self.disable_key_order_spin_box(redistribute=True))
        self.ui.brakeCheckBox.stateChanged.connect(lambda : self.disable_key_order_spin_box(redistribute=True))

        # Connections fot the pan and tilt settings toolbox
        self.ui.panTiltSetButton.clicked.connect(self.pan_tilt_config_set_clicked)
        self.ui.panTiltResetButton.clicked.connect(self.pan_tilt_config_reset_clicked)

        self.ui.panSensivitySlider.valueChanged.connect(self.pan_sensitivity_slider_changed)
        self.ui.panSensitivitySpinBox.valueChanged.connect(self.pan_sensitivity_spinbox_changed)
        self.ui.tiltSensivitySlider.valueChanged.connect(self.tilt_sensitivity_slider_changed)
        self.ui.tiltSensivitySpinBox.valueChanged.connect(self.tilt_sensitivity_spinbox_changed)

        self.ui.panTiltEnableCheckBox.stateChanged.connect(self.enable_pan_tilt_checkbox_changed)

        # Connection for the gas sensor alarm settings
        self.ui.gasThreshSetButton.clicked.connect(self.gas_threshold_set_clicked)
        self.ui.gasThreshResetButton.clicked.connect(self.gas_threshold_reset_clicked)

        # Connection for the particles sensor alarm settings
        self.ui.partThreshSetButton.clicked.connect(self.particle_threshold_set_clicked)
        self.ui.partThreshResetButton.clicked.connect(self.particle_threshold_reset_clicked)

        # Connection for the SMS alert settings
        self.ui.smsAlertSetButton.clicked.connect(self.sms_alert_set_clicked)
        self.ui.smsAlertResetButton.clicked.connect(self.sms_alert_reset_clicked)

        self.ui.smsAlertEnableCheckBox.stateChanged.connect(self.enable_sms_alert_checkbox_changed)

        # Connection for the main menu on the menu bar
        self.ui.menuMenu.triggered[QtGui.QAction].connect(self.process_menu_trigger)

        # Connection for the help menu
        self.ui.menuHelp.triggered[QtGui.QAction].connect(self.process_help_trigger)

        # Connection for warning flasher timer signal
        self.warning_flasher_timer = QtCore.QTimer()
        self.warning_flasher_timer.timeout.connect(self.hud_warning_flasher_set)
        self.warning_flasher_timer.setInterval(self.flash_time_w)

        # Connection to non critical information flasher timer signal
        self.non_critical_flasher_timer=QtCore.QTimer()
        self.non_critical_flasher_timer.timeout.connect(self.hud_non_critical_flasher_set)
        self.non_critical_flasher_timer.setInterval(self.flash_timer_nc)

        # Connection to the DNN output key press signal
        # self.dnn.dnnOutputKeyPress.connect(self.handle_dnn_key_press)

    def attributes_of_ui(self):  # Properties of the GUI

        # For info display text edit
        self.ui.infoDisplay.setReadOnly(True)  # The text in the info is read only

        # For the connection setting toolbox
        self.ui.dataPortSpinBox.setMinimum(0)
        self.ui.dataPortSpinBox.setMaximum(65535)

        self.ui.videoPortSpinBox.setMinimum(0)
        self.ui.videoPortSpinBox.setMaximum(65535)

        # For video feed settings
        self.ui.frameRateSpinBox.setMinimum(5)
        self.ui.frameRateSpinBox.setMaximum(60)

        # For video recording settings
        self.ui.videoRecordingFPSSpinBox.setMinimum(5)
        self.ui.videoRecordingFPSSpinBox.setMaximum(120)

        # For the Training data recording
        self.ui.autoSaveSampleSpinBox.setMinimum(10)
        self.ui.autoSaveSampleSpinBox.setMaximum(10000)

        self.ui.imgWidthSpinBox.setMinimum(5)
        self.ui.imgWidthSpinBox.setMaximum(1000)

        self.ui.imgHeightSpinBox.setMinimum(5)
        self.ui.imgHeightSpinBox.setMaximum(1000)

        # For the key order spin boxes
        self.ui.forwardOrderSpinBox.setRange(0,self.keys_comb_len)
        self.ui.backwardOrderSpinBox.setRange(0,self.keys_comb_len)
        self.ui.leftTurnOrderSpinBox.setRange(0,self.keys_comb_len)
        self.ui.rightTurnOrderSpinBox.setRange(0,self.keys_comb_len)
        self.ui.l_turn_fOrderSpinBox.setRange(0,self.keys_comb_len)
        self.ui.r_turn_fOrderSpinBox.setRange(0,self.keys_comb_len)
        self.ui.l_turn_bOrderSpinBox.setRange(0,self.keys_comb_len)
        self.ui.r_turn_bOrderSpinBox.setRange(0,self.keys_comb_len)
        self.ui.acclOrderSpinBox.setRange(0,self.keys_comb_len)
        self.ui.brakeOrderSpinBox.setRange(0,self.keys_comb_len)

        # For camera pan and tilt settings
        self.ui.panLeftSpinBox.setRange(0,180)
        self.ui.panRightSpinBox.setRange(0,180)
        self.ui.tiltDownSpinBox.setRange(0,180)
        self.ui.tiltUpSpinBox.setRange(0,180)

        self.ui.panSensivitySlider.setRange(1,self.max_pan_tilt_scaler)
        self.ui.panSensivitySlider.setTickInterval(1)

        self.ui.tiltSensivitySlider.setRange(1,self.max_pan_tilt_scaler)
        self.ui.tiltSensivitySlider.setTickInterval(1)

        self.ui.panSensitivitySpinBox.setRange(1, self.max_pan_tilt_scaler)
        self.ui.tiltSensivitySpinBox.setRange(1,self.max_pan_tilt_scaler)

        # For the video display settings
        self.ui.blendSlider.setMinimum(0)
        self.ui.blendSlider.setMaximum(100)
        self.ui.blendSlider.setValue(50)

        self.ui.thermBlendScalerSlider.setMinimum(1)
        self.ui.thermBlendScalerSlider.setMaximum(125)
        self.ui.thermBlendScalerSlider.setValue(1)

        self.ui.thermPosHorzSlider.setMinimum(-500)
        self.ui.thermPosHorzSlider.setMaximum(500)
        self.ui.thermPosHorzSlider.setTickInterval(40)
        self.ui.thermPosHorzSlider.setValue(0)

        self.ui.thermPosVertSlider.setMinimum(-500)
        self.ui.thermPosVertSlider.setMaximum(500)
        self.ui.thermPosVertSlider.setTickInterval(40)
        self.ui.thermPosVertSlider.setValue(0)

        # For sensor alarm settings toolbox
        self.ui.eCO2ThresholdSpinBox.setMinimum(0)
        self.ui.eCO2ThresholdSpinBox.setMaximum(1000000)

        self.ui.tvocThresholdSpinBox.setMinimum(0)
        self.ui.tvocThresholdSpinBox.setMaximum(1000000000)

        self.ui.redThresholdSpinBox.setMinimum(0)
        self.ui.redThresholdSpinBox.setMaximum(1000000)

        self.ui.greenThresholdSpinBox.setMinimum(0)
        self.ui.greenThresholdSpinBox.setMaximum(1000000)

        self.ui.irThresholdSpinBox.setMinimum(0)
        self.ui.irThresholdSpinBox.setMaximum(1000000)

        # For the SMS alert settings
        self.ui.smsNumberLineEdit.setMaxLength(14)  # A maximum of 14 character is allowed

    def display_info(self,msg_id,disp_string):  # Function to print information to the text display
        msg_time=datetime.datetime.now().strftime('[%H:%M:%S]')
        final_string="["+msg_id+"] "+msg_time+": "+disp_string
        self.ui.infoDisplay.append(final_string)

    # Socket connection
    def establish_connection(self):  # Function to establish the connection (Called when connect is clicked)

        self.update_connection_info_bar()  # update the display bar

        if self.ui.connectButton.text() == "Connect":  # User trying to connect

            self.reinitialize_sockets_thread()  # Reinitialize the connection with the host, port and type

            self.display_info(self.app_msg,"Trying to connect to specified ports...")
            self.display_info(self.app_msg,"Host IP: "+str(self.host))
            self.display_info(self.app_msg,"Data port: "+str(self.port_data))
            self.display_info(self.app_msg,"Video port: "+str(self.port_img))

            self.ui.connectButton.setText("Connecting...")  # Set the text of the button to connection this is used

            self.soc_data.start_sock_read_thread()  # Socket for sending data is started
            self.soc_img.start_sock_read_thread()  # Socket for video capture is started

            self.ui.dataSocketStatusLabel.setText("Disconnected")
            self.ui.capSocketStatusLabel.setText("Disconnected")

        elif self.ui.connectButton.text() == "Connecting...":  # Used to cancel the connection that is in progress
            self.display_info(self.app_msg,"Connection attempt was cancelled by the user")

            self.ui.dataSocketStatusLabel.setText("Disconnected")
            self.ui.capSocketStatusLabel.setText("Disconnected")

            self.close_socket_routine()  # Function to close the sockets (Button text changed in function)

        elif self.ui.connectButton.text() == "Disconnect":  # To disconnect the above procedure is used again
            self.display_info(self.app_msg,"Disconnecting from the robot")

            # Since we are closing the sockets manually, we set the socket connection status as false
            self.socket_data_connected=False
            self.socket_img_connected=False

            self.ui.dataSocketStatusLabel.setText("Disconnected")
            self.ui.capSocketStatusLabel.setText("Disconnected")

            self.close_socket_routine()  # Close the socket

    def close_socket_routine(self):  # Function to close the socket communication and reinitialize

        # Shutdown the sockets first to let the client know we are disconnecting
        self.soc_data.socket_shutdown()
        self.soc_img.socket_shutdown()

        # Close the threads if they are running
        self.soc_data.stop_sock_read_thread()
        self.soc_img.stop_sock_read_thread()

        # Since we closed the sockets in the above section we need to open new ports
        self.reinitialize_sockets_thread()

        # After everything is done set the text to connect, to allow the user to connect again
        self.ui.connectButton.setText("Connect")
        self.display_black_screen()

    def reinitialize_sockets_thread(self):  # Function to reinitialize the socket and thread to start a new one

        # Reinitialize the class once again
        self.soc_data.__init__(self.host, self.port_data, SockComm.DATA_TYPE)
        self.soc_img.__init__(self.host, self.port_img, SockComm.IMG_TYPE)

        # When we reinitialize the class the signal connections are lost and therefore needs to reconnected
        # Signal for the new frame received from the MJPEG video stream
        self.soc_img.frameReceived.connect(self.get_socket_stream_frames)
        self.soc_img.capStatus.connect(self.capture_status_action)

        # Signals for the new data received
        self.soc_data.dataReceived.connect(self.get_socket_stream_data)
        self.soc_data.sockConnected.connect(self.data_status_action)

        print("Socket communication reinitialization done!")

    def retrieve_configurations(self):  # Function to retrieve configurations from the RPi side

        # For the camera settings
        cmd=self.format_command(self.socket_send_header_dict['RETRIEVE'],
                                [self.socket_send_header_dict['CAM_SET']])

        self.send_socket_commands(cmd)

        # For the gas threshold settings
        cmd=self.format_command(self.socket_send_header_dict['RETRIEVE'],
                                [self.socket_send_header_dict['GAS_THR']])

        self.send_socket_commands(cmd)

        # For the particle threshold settings
        cmd=self.format_command(self.socket_send_header_dict['RETRIEVE'],
                                [self.socket_send_header_dict['PAR_THR']])

        self.send_socket_commands(cmd)

        # For the SMS settings
        cmd=self.format_command(self.socket_send_header_dict['RETRIEVE'],
                                [self.socket_send_header_dict['SMS']])

        self.send_socket_commands(cmd)

    # Functions to get the host and ports from user
    def connection_save_clicked(self):
        self.host=self.ui.hostLineEdit.text()  # Get the text (Not sanitized)

        self.port_data=self.ui.dataPortSpinBox.value()
        self.port_img=self.ui.videoPortSpinBox.value()

        self.config.set('Connection','host',str(self.host))
        self.config.set('Connection','data_port',str(self.port_data))
        self.config.set('Connection','video_port',str(self.port_img))

        self.write_to_config_file()

        cmd=self.format_command(self.socket_send_header_dict['COM_SET'],[self.host,self.port_data,self.port_img])

        con_str=""

        if self.ui.saveToClientcheckBox.isChecked():  # If the save to client is checked other changes are made locally
            self.send_socket_commands(cmd)  # Send the command to change the host and the ports
            self.ui.saveToClientcheckBox.setChecked(False)
            con_str = "The connection setting will be sent to client"
        else:
            con_str = "Only applied locally"

        self.display_info(self.app_msg, "New connection settings saved. Will be used on next reconnect. "+con_str)

    def connection_reset_clicked(self):

        self.get_connection_settings_cfg()

        self.ui.hostLineEdit.setText(str(self.host))
        self.ui.dataPortSpinBox.setValue(self.port_data)
        self.ui.videoPortSpinBox.setValue(self.port_img)

        self.display_info(self.app_msg,"The connection settings have been reset to previous values")

    def get_connection_settings_cfg(self):  # Function to get connection setting from config file
        self.host = self.config.get('Connection', 'host')
        self.port_data = self.config.getint('Connection', 'data_port')
        self.port_img = self.config.getint('Connection', 'video_port')

    def update_connection_info_bar(self):  # Function to set the connection text under the video display area
        self.ui.hostLabel.setText(str(self.host))
        self.ui.dataPortLabel.setText(str(self.port_data))
        self.ui.videoPortLabel.setText(str(self.port_img))

    def update_host_and_ports_input_section(self):  # Function to update the user input connection settings
        self.ui.hostLineEdit.setText(str(self.host))
        self.ui.dataPortSpinBox.setValue(self.port_data)
        self.ui.videoPortSpinBox.setValue(self.port_img)

    # Deep Neural Network support functions
    def neural_net_activate_deactivate(self):

        if self.socket_img_connected is True:  # If the socket for video frame is connected

            self.nn_active = not self.nn_active

            if self.nn_active is True:
                self.display_info(self.app_msg,"Deep Nerual Network activated")

            else:
                self.display_info(self.app_msg,"Deep Nerual Network deactivated")
                self.key_pressed_dict=self.default_key_pressed_dict.copy()  # Shallow copy is used to reset the dict

        else:
            self.display_info(self.app_msg, "Deep Neural Network cannot be activated. No video feed available")

    # Capture stream functions
    def get_socket_stream_frames(self, jpeg_bytes, therm_jpeg_bytes):  # Gets the frames and updates the display

        # This is the normal RGB camera image
        if jpeg_bytes is not None:
            if len(jpeg_bytes) > 0:

                temp_frame_bgr=cv2.imdecode(np.fromstring(jpeg_bytes,dtype=np.uint8),cv2.IMREAD_COLOR)  # convert string bytes to image

                if temp_frame_bgr is not None:  # If the above operation did not produced a none
                    self.frame_bgr=temp_frame_bgr  # The main frame for BGR data is updated

                # Training images are collected from here
                """
                Training image data is collected from here because if the resized image
                of the final frame was resized to smaller size some of the error caused
                during the initialize the resizing can cause loss of detail in the image,
                although minor can be avoided if the original frame is resized.
                """

                if self.enable_training_data_recording:  # This produces the smaller frame of the current frame
                    self.frame_train_bgr=cv2.resize(self.frame_bgr, (self.train_frame_width, self.train_frame_height),
                                                    interpolation=cv2.INTER_LINEAR)

                if self.nn_active:  # Frames for the input to the neural network
                    self.frame_dnn_bgr=cv2.resize(self.frame_bgr,(80,60),interpolation=cv2.INTER_LINEAR)

                height, width = self.frame_bgr.shape[:2]  # Get the size of the image

                if width != 1280 and height != 720:  # Resize the image to fit display area
                    self.frame_bgr=cv2.resize(self.frame_bgr,(1280, 720),interpolation=cv2.INTER_LINEAR)

        # Thermal camera image
        if therm_jpeg_bytes is not None:
            if len(therm_jpeg_bytes) > 0:  # This avoids corrupted or null byte strings

                temp_frame_thermal=cv2.imdecode(np.fromstring(therm_jpeg_bytes,dtype=np.uint8),cv2.IMREAD_COLOR)

                if temp_frame_thermal is not None:  # If the above operation did not produce a none
                    self.frame_thermal=temp_frame_thermal  # The main frame for thermal data is updated

                self.frame_thermal_copy=self.frame_thermal.copy()  # Copy the original frame for blend purposes
                height, width = self.frame_thermal.shape[:2]  # Get the size of the image

                if width != 1280 and height != 720:
                    self.frame_thermal=cv2.resize(self.frame_thermal,(1280, 720),interpolation=cv2.INTER_LINEAR) # Resize the image to fit display area

        # Display the image on the GUI according to the source selected
        if self.normal_cam_enable is True and self.thermal_cam_enable is True:  # Blend of two sources
            aspect_changed=self.change_to_16by9_thermal_image(self.frame_thermal_copy)
            scaled_result=self.scale_thermal_image(aspect_changed, self.therm_scale_val,scale_divider=10)
            padded_result=self.pad_thermal_image(scaled_result)
            trans_result=self.translate_thermal_image(padded_result,self.therm_pos_horz,self.therm_pos_vert)

            dst=cv2.addWeighted(self.frame_bgr,float(self.norm_cam_weight/100.0),
                                    trans_result,float(self.therm_cam_weight/100.0),0)

            self.final_frame=dst

        elif self.normal_cam_enable is True:  # Just the normal camera
            self.final_frame=self.frame_bgr

        elif self.thermal_cam_enable is True:  # Just the thermal camera
            self.final_frame=self.frame_thermal

        # The video frame update was normally done in this function but was moved to the update video feed function

    def update_video_feed_display(self):  # Function to update the video feed at whatever frame rate

        if self.socket_img_connected is True:
            frame=self.final_frame.copy()  # Copy the frame
            self.video_HUD_display(frame)  # Add the HUD elements to the frame

            # Giving the frame to the Neural Network
            # if self.nn_active is True:
            #     self.dnn.dnn_model_prediction(dnn_input=self.frame_dnn_bgr)

            # The HUD elements are also saved
            if self.enable_video_recording is True and not self.video_recording_paused:
                # The video recording file is automatically opened in the below function
                retval=self.vid_saver.save_frames_to_video_file(frame,timestamped=True)  # Separate recording each time
                self.ui.frameCountLabel.setText(str(self.vid_saver.get_frame_count()))
                if retval is False:  # If the recording is malfunctioning
                    self.display_info(self.app_msg,"Video recording failed, recording stopped")
                    self.enable_video_recording=False  # Stop the recording
                    # The button state has to be changed here as well

            # The training data is collected here
            if self.enable_training_data_recording and not self.training_data_recording_paused:
                if self.is_keys_active():  # If any of the keys are pressed
                    key_inputs=self.output_key_presses()
                    if not self.is_disabled_key_pressed():  # If none of the disabled keys are pressed then save
                        self.save_done, self.training_frame_count=self.train_data_saver.save_training_data(
                                                                    train_input=self.frame_train_bgr,
                                                                    train_label=key_inputs)

                        self.ui.trainingExampleCountLabel.setText(str(self.training_frame_count))

            pix_frame=self.convert_frame_to_pix(frame)  # Convert for the pix format required by Qt
            self.ui.video_frame.setPixmap(pix_frame)  # Finally update the video frame

        else:
            self.display_black_screen()  # Display the black screen

    def calc_interval_from_FPS(self):  # Function that calculates the interval for display update from FPS
        interval=(1/float(self.display_fps))*1000.0  # Interval in millisecond
        interval=int(round(interval,0))

        return interval  # The interval between frame updates

    @staticmethod
    def convert_frame_to_pix(frame_bgr): # Function to convert cv2 frame to pix format to be displayed in GUI
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)  # BGR to RGB
        image = QtGui.QImage(frame_rgb, frame_rgb.shape[1], frame_rgb.shape[0], QtGui.QImage.Format_RGB888)
        pixel = QtGui.QPixmap.fromImage(image)
        return pixel  # The pix formatted image ready to be displayed

    def video_HUD_display(self, frame_bgr):  # Function to display Heads Up Display (HUD) elements

        if frame_bgr is not None and self.ui.enableHUDCheckBox.isChecked():
            if len(frame_bgr) > 0:
                height, width=frame_bgr.shape[:2]

                # CENTER part of the video display
                # Drawing a cross-hair
                cross_hair_size=40  # In pixels
                cross_hair_thickness=1  # In pixels
                cross_hair_color=(0,255,0)

                cv2.line(frame_bgr, (int((width / 2) - cross_hair_size), int(height / 2)),
                         (int((width / 2) + cross_hair_size), int(height / 2)), cross_hair_color, cross_hair_thickness)

                cv2.line(frame_bgr, (int((width / 2)), int((height / 2) - cross_hair_size)),
                         (int((width / 2)), int((height / 2) + cross_hair_size)), cross_hair_color, cross_hair_thickness)

                # RIGHT part of the video display
                # Display sensor readings on HUD
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.45
                font_thickness = 1
                font_color = (0, 255, 0)

                RIGHT_MAX_TEXT = "XXXXX: 888888.88"
                retval, baseline = cv2.getTextSize(RIGHT_MAX_TEXT, font, font_scale, font_thickness)
                right_text_pw = width - retval[0]-10  # Place width
                right_text_ph = 50  # Place height
                text_line_space_r = baseline + retval[1]
                text_vert_offset_r=0

                string = "SENSOR DATA"
                cv2.putText(frame_bgr, string, (right_text_pw, right_text_ph + text_vert_offset_r),
                            font, font_scale, font_color, font_thickness, cv2.LINE_AA)

                # Gas sensor
                string="eCO2: "+str(self.eCO2)
                text_vert_offset_r = text_vert_offset_r + 2*text_line_space_r
                cv2.putText(frame_bgr, string, (right_text_pw, right_text_ph + text_vert_offset_r),
                            font, font_scale, font_color, font_thickness,cv2.LINE_AA)

                string="TVOC: "+str(self.TVOC)
                text_vert_offset_r=text_vert_offset_r+text_line_space_r
                cv2.putText(frame_bgr, string, (right_text_pw, right_text_ph + text_vert_offset_r),
                            font, font_scale, font_color, font_thickness,cv2.LINE_AA)

                # Environmental sensor
                string="PRESS: "+str(self.pressure)
                text_vert_offset_r = text_vert_offset_r + 2*text_line_space_r
                cv2.putText(frame_bgr, string, (right_text_pw, right_text_ph + text_vert_offset_r),
                            font, font_scale, font_color, font_thickness, cv2.LINE_AA)

                string="TEMP: "+str(self.temperature)
                text_vert_offset_r = text_vert_offset_r + text_line_space_r
                cv2.putText(frame_bgr, string, (right_text_pw, right_text_ph + text_vert_offset_r),
                            font, font_scale, font_color, font_thickness, cv2.LINE_AA)

                string="HUM: "+str(self.humidity)
                text_vert_offset_r = text_vert_offset_r + text_line_space_r
                cv2.putText(frame_bgr, string, (right_text_pw, right_text_ph + text_vert_offset_r),
                            font, font_scale, font_color, font_thickness, cv2.LINE_AA)

                # Particle sensor
                string="RED: "+str(self.red)
                text_vert_offset_r = text_vert_offset_r + 2*text_line_space_r
                cv2.putText(frame_bgr, string, (right_text_pw, right_text_ph + text_vert_offset_r),
                            font, font_scale, font_color, font_thickness, cv2.LINE_AA)

                string="GREEN: "+str(self.green)
                text_vert_offset_r = text_vert_offset_r + text_line_space_r
                cv2.putText(frame_bgr, string, (right_text_pw, right_text_ph + text_vert_offset_r),
                            font, font_scale, font_color, font_thickness, cv2.LINE_AA)

                string="IR: "+str(self.ir)
                text_vert_offset_r = text_vert_offset_r + text_line_space_r
                cv2.putText(frame_bgr, string, (right_text_pw, right_text_ph + text_vert_offset_r),
                            font, font_scale, font_color, font_thickness, cv2.LINE_AA)

                # TOP part of the video display
                # Status display on HUD
                TOP_MAX_TEXT="XXXXXXXXXX"
                top_text_ph=50
                top_text_pw=50
                prev_text_size=0
                text_gap=40
                text_horz_offset=0
                box_spacer=3

                # Time display
                current_time = datetime.datetime.now().strftime('[%H:%M:%S]')
                cv2.putText(frame_bgr, current_time, (top_text_pw, top_text_ph),
                            font, font_scale, font_color, font_thickness, cv2.LINE_AA)
                prev_text_size,_=cv2.getTextSize(TOP_MAX_TEXT, font, font_scale, font_thickness)
                text_horz_offset=text_horz_offset+prev_text_size[0]+text_gap

                # Link quality
                string="LINK: "+str(self.wifi_link_quality)
                cv2.putText(frame_bgr,string,(top_text_pw+text_horz_offset,top_text_ph),
                            font,font_scale,font_color,font_thickness,cv2.LINE_AA)
                prev_text_size,_=cv2.getTextSize(TOP_MAX_TEXT, font, font_scale, font_thickness)
                text_horz_offset=text_horz_offset+prev_text_size[0]+text_gap

                # Data socket connection status
                stat_str=""
                if self.socket_data_connected is True:
                    stat_str="OK"
                else:
                    stat_str="?"

                string="DATA: "+stat_str
                cv2.putText(frame_bgr, string, (top_text_pw + text_horz_offset, top_text_ph),
                            font, font_scale, font_color, font_thickness, cv2.LINE_AA)
                prev_text_size, _ = cv2.getTextSize(TOP_MAX_TEXT, font, font_scale, font_thickness)
                text_horz_offset = text_horz_offset + prev_text_size[0] + text_gap

                # Microcontroller status
                if self.micro_controller_rdy is True:
                    stat_str="RDY"
                else:
                    stat_str="?"

                string="MCU: "+stat_str
                cv2.putText(frame_bgr, string, (top_text_pw + text_horz_offset, top_text_ph),
                            font, font_scale, font_color, font_thickness, cv2.LINE_AA)
                prev_text_size, _ = cv2.getTextSize(TOP_MAX_TEXT, font, font_scale, font_thickness)
                text_horz_offset = text_horz_offset + prev_text_size[0] + text_gap

                # Neural network active
                string = "NEURAL NET ACTIVE"
                # The string has to be used to calculate the rectangle around the text
                prev_text_size, baseline = cv2.getTextSize(string, font, font_scale, font_thickness)

                if self.nn_active:  # Show the warning in HUD if threshold is reached
                    cv2.putText(frame_bgr, string, (top_text_pw + text_horz_offset, top_text_ph),
                                font, font_scale, font_color, font_thickness, cv2.LINE_AA)

                    cv2.rectangle(frame_bgr, (top_text_pw + text_horz_offset - box_spacer, top_text_ph - prev_text_size[1] - baseline),
                                  (top_text_pw + text_horz_offset + prev_text_size[0] + box_spacer, top_text_ph + baseline), font_color, 1)

                text_horz_offset = text_horz_offset + prev_text_size[0] + text_gap

                # Warnings for the data coming from the sensor
                # Gas threshold reached warning
                string="GAS WARNING"
                # The string has to be used to calculate the rectangle around the text
                prev_text_size, baseline = cv2.getTextSize(string, font, font_scale, font_thickness)

                if self.gas_threshold_reached and self.flash_warning:  # Show the warning in HUD if threshold is reached
                    cv2.putText(frame_bgr, string, (top_text_pw + text_horz_offset, top_text_ph),
                                font, font_scale, font_color, font_thickness, cv2.LINE_AA)

                    cv2.rectangle(frame_bgr, (top_text_pw + text_horz_offset - box_spacer, top_text_ph - prev_text_size[1] - baseline),
                                  (top_text_pw + text_horz_offset + prev_text_size[0] + box_spacer, top_text_ph + baseline), font_color, 1)

                text_horz_offset = text_horz_offset + prev_text_size[0] + text_gap

                # Particle threshold reached warning
                string = "PARTICLE WARNING"
                # The string has to be used to calculate the rectangle around the text
                prev_text_size, baseline = cv2.getTextSize(string, font, font_scale, font_thickness)

                if self.particle_threshold_reached and self.flash_warning:  # Show the warning in HUD if threshold is reached
                    cv2.putText(frame_bgr, string, (top_text_pw + text_horz_offset, top_text_ph),
                                font, font_scale, font_color, font_thickness, cv2.LINE_AA)

                    cv2.rectangle(frame_bgr, (top_text_pw + text_horz_offset - box_spacer, top_text_ph - prev_text_size[1] - baseline),
                                  (top_text_pw + text_horz_offset + prev_text_size[0] + box_spacer, top_text_ph + baseline), font_color, 1)

                text_horz_offset = text_horz_offset + prev_text_size[0] + text_gap

                # LEFT part of the video display
                # Camera type and FPS information
                LEFT_MAX_TEXT="XXXXXXXX"
                retval, baseline = cv2.getTextSize(LEFT_MAX_TEXT, font, font_scale, font_thickness)  # Get an estimate
                text_line_space_l = baseline + retval[1]  # The gap between two lines (Y-direction)
                text_vert_offset_l = 0  # This has to be calculated and updated with the addition of new text
                box_spacer = 3  # spacing of 3 pixels
                left_text_pw = 50+box_spacer  # Place width
                left_text_ph = 50+(4*text_line_space_l)  # Place height

                if self.normal_cam_enable and self.thermal_cam_enable:  # Display info to HUD as overlay
                    string="OVRLAY"
                elif self.normal_cam_enable is True:  # Normal camera
                    string="NORMAL"
                elif self.thermal_cam_enable is True:  # Thermal FLIR camera
                    string="FLIR"

                # The string has to be used to calculate the rectangle around the text
                prev_text_size, baseline = cv2.getTextSize(string, font, font_scale, font_thickness)

                cv2.putText(frame_bgr, string, (left_text_pw, left_text_ph + text_vert_offset_l),
                            font, font_scale, font_color, font_thickness, cv2.LINE_AA)

                cv2.rectangle(frame_bgr, (left_text_pw + text_vert_offset_l - box_spacer, left_text_ph - prev_text_size[1] - baseline),
                              (left_text_pw + text_vert_offset_l + prev_text_size[0] + box_spacer, left_text_ph + baseline), font_color, 1)

                string=str(self.display_fps)+" Hz"
                text_vert_offset_l = text_vert_offset_l + baseline + text_line_space_l + 2
                cv2.putText(frame_bgr, string, (left_text_pw - box_spacer, left_text_ph + text_vert_offset_l),
                            font, font_scale, font_color, font_thickness, cv2.LINE_AA)

                # BOTTOM part of the video display
                BOTTOM_MAX_TEXT="XXXXXXXXX"
                btm_text_ph = height-50
                btm_text_pw = 50
                prev_text_size = 0
                text_gap = 40
                text_horz_offset = 0
                box_spacer = 3

                # Video recording status
                string = "REC ACT"
                if self.video_recording_paused is True:
                    string = "REC PSD"

                # The string has to be used to calculate the rectangle around the text
                prev_text_size, baseline = cv2.getTextSize(string, font, font_scale, font_thickness)

                if self.enable_video_recording and self.flash_non_critical:  # Show the warning in HUD if threshold is reached
                    cv2.putText(frame_bgr, string, (btm_text_pw + text_horz_offset, btm_text_ph),
                                font, font_scale, font_color, font_thickness, cv2.LINE_AA)

                    cv2.rectangle(frame_bgr, ( btm_text_pw + text_horz_offset - box_spacer, btm_text_ph - prev_text_size[1] - baseline),
                                  (btm_text_pw + text_horz_offset + prev_text_size[0] + box_spacer,
                                   btm_text_ph + baseline), font_color, 1)

                text_horz_offset = text_horz_offset + prev_text_size[0] + text_gap

                # Training data recording status
                string = "TD REC ACT"
                if self.training_data_recording_paused is True:
                    string = "TD REC PSD"

                # The string has to be used to calculate the rectangle around the text
                prev_text_size, baseline = cv2.getTextSize(string, font, font_scale, font_thickness)

                if self.enable_training_data_recording and self.flash_non_critical:  # Show the warning in HUD if threshold is reached
                    cv2.putText(frame_bgr, string, (btm_text_pw + text_horz_offset, btm_text_ph),
                                font, font_scale, font_color, font_thickness, cv2.LINE_AA)

                    cv2.rectangle(frame_bgr, ( btm_text_pw + text_horz_offset - box_spacer, btm_text_ph - prev_text_size[1] - baseline),
                                  (btm_text_pw + text_horz_offset + prev_text_size[0] + box_spacer,
                                   btm_text_ph + baseline), font_color, 1)

                text_horz_offset = text_horz_offset + prev_text_size[0] + text_gap

                # Disabled key pressed
                string = "DISB KEY PRESS"

                # The string has to be used to calculate the rectangle around the text
                prev_text_size, baseline = cv2.getTextSize(string, font, font_scale, font_thickness)

                if self.disabled_move_key_pressed is True:  # If any of the disabled key is pressed
                    cv2.putText(frame_bgr, string, (btm_text_pw + text_horz_offset, btm_text_ph),
                                font, font_scale, font_color, font_thickness, cv2.LINE_AA)

                    cv2.rectangle(frame_bgr, (btm_text_pw + text_horz_offset - box_spacer, btm_text_ph - prev_text_size[1] - baseline),
                                  (btm_text_pw + text_horz_offset + prev_text_size[0] + box_spacer,
                                   btm_text_ph + baseline), font_color, 1)

                    self.disabled_move_key_pressed=False  # Reset the variable

                text_horz_offset = text_horz_offset + prev_text_size[0] + text_gap


                # BOTTOM CENTER of the video display
                # Variables for the text
                btc_text_ph = height - 50
                btc_text_pw = int(width/2)
                prev_text_size = 0
                text_gap = 40
                text_horz_offset = 0
                box_spacer = 3

                # Variables for the arrow
                cpo=50  # Center place offset
                offset=10  # Offset from the center place of the arrow start
                arrow_length=30  # Length of the arrow

                # For acceleration
                if self.key_pressed_dict['SHIFT']:
                    string = "ACCEL"
                    # The string has to be used to calculate the rectangle around the text
                    prev_text_size, baseline = cv2.getTextSize(string, font, font_scale, font_thickness)

                    text_horz_offset= - offset - arrow_length - prev_text_size[0] - 4*box_spacer

                    cv2.putText(frame_bgr, string, (btc_text_pw + text_horz_offset, btc_text_ph),
                                font, font_scale, font_color, font_thickness, cv2.LINE_AA)

                    cv2.rectangle(frame_bgr, (btc_text_pw + text_horz_offset - box_spacer, btc_text_ph - prev_text_size[1] - baseline),
                                  (btc_text_pw + text_horz_offset + prev_text_size[0] + box_spacer,
                                   btc_text_ph + baseline), font_color, 1)

                # For braking
                if self.key_pressed_dict['CTRL']:  # Braking
                    string = "BRAKE"
                    # The string has to be used to calculate the rectangle around the text
                    prev_text_size, baseline = cv2.getTextSize(string, font, font_scale, font_thickness)

                    text_horz_offset = offset + arrow_length + 4*box_spacer

                    cv2.putText(frame_bgr, string, (btc_text_pw + text_horz_offset, btc_text_ph),
                                font, font_scale, font_color, font_thickness, cv2.LINE_AA)

                    cv2.rectangle(frame_bgr, (btc_text_pw + text_horz_offset - box_spacer, btc_text_ph - prev_text_size[1] - baseline),
                                  (btc_text_pw + text_horz_offset + prev_text_size[0] + box_spacer,
                                   btc_text_ph + baseline), font_color, 1)

                if not self.key_pressed_dict['A'] and not self.key_pressed_dict['D']:  # Forward/Backward
                    # Forward
                    if self.key_pressed_dict['W']:
                        cv2.arrowedLine(frame_bgr, (int(width / 2), height - cpo - offset),
                                        (int(width / 2), height - cpo - offset - arrow_length)
                                        , font_color, 1, cv2.LINE_AA, tipLength=0.5)

                    # Backward
                    elif self.key_pressed_dict['S']:
                        cv2.arrowedLine(frame_bgr, (int(width / 2), height - cpo + offset),
                                        (int(width / 2), height - cpo + offset + arrow_length)
                                        , font_color, 1, cv2.LINE_AA, tipLength=0.5)

                elif not self.key_pressed_dict['W'] and not self.key_pressed_dict['S']:  # Left/Right
                    # Left
                    if self.key_pressed_dict['A']:
                        cv2.arrowedLine(frame_bgr, (int(width / 2) - offset, height - cpo),
                                        (int(width / 2) - offset - arrow_length, height - cpo)
                                        , font_color, 1, cv2.LINE_AA, tipLength=0.5)

                    # Right
                    elif self.key_pressed_dict['D']:
                        cv2.arrowedLine(frame_bgr, (int(width / 2) + offset, height - cpo),
                                        (int(width / 2) + offset + arrow_length, height - cpo)
                                        , font_color, 1, cv2.LINE_AA, tipLength=0.5)

                # Forward and left
                elif self.key_pressed_dict['W'] and self.key_pressed_dict['A']:
                    cv2.arrowedLine(frame_bgr, (int(width / 2) - int(offset/1.414), height - cpo - int(offset/1.414)),
                                    (int(width / 2) + int((-offset - arrow_length)/1.414), height - cpo + int((-offset - arrow_length)/1.414))
                                    , font_color, 1, cv2.LINE_AA, tipLength=0.5)

                # Forward and right
                elif self.key_pressed_dict['W'] and self.key_pressed_dict['D']:
                    cv2.arrowedLine(frame_bgr, (int(width / 2) + int(offset / 1.414), height - cpo - int(offset / 1.414)),
                                    (int(width / 2) - int((-offset - arrow_length) / 1.414),
                                     height - cpo + int((-offset - arrow_length) / 1.414))
                                    , font_color, 1, cv2.LINE_AA, tipLength=0.5)

                # Backward and left
                elif self.key_pressed_dict['S'] and self.key_pressed_dict['A']:
                    cv2.arrowedLine(frame_bgr, (int(width / 2) - int(offset / 1.414), height - cpo + int(offset / 1.414)),
                                    (int(width / 2) + int((-offset - arrow_length) / 1.414),
                                     height - cpo - int((-offset - arrow_length) / 1.414))
                                    , font_color, 1, cv2.LINE_AA, tipLength=0.5)

                # Backward and right
                elif self.key_pressed_dict['S'] and self.key_pressed_dict['D']:
                    cv2.arrowedLine(frame_bgr, (int(width / 2) + int(offset / 1.414), height - cpo + int(offset / 1.414)),
                                    (int(width / 2) - int((-offset - arrow_length) / 1.414),
                                     height - cpo - int((-offset - arrow_length) / 1.414))
                                    , font_color, 1, cv2.LINE_AA, tipLength=0.5)

    def hud_warning_flasher_set(self):  # Function that determines when to blink the warning on hud
        self.flash_warning = not self.flash_warning

    def hud_non_critical_flasher_set(self):  # Function that determines when to blink the non critical warning
        self.flash_non_critical = not self.flash_non_critical

    def recording_start_stop(self):  # Function to start/stop recording (Just a test function)

        if self.socket_img_connected is True:  # Recording is only enabled if video feed is active

            self.enable_video_recording=not self.enable_video_recording

            if self.enable_video_recording is True:
                if not self.non_critical_flasher_timer.isActive():
                    self.non_critical_flasher_timer.start()

                # The recording file is opened automatically in save video frame function
                self.ui.startVideoRecButton.setText("Recording...")  # Set the button text
                self.display_info(self.app_msg,"Video recording has started")
            else:
                if self.non_critical_flasher_timer.isActive() and not self.enable_training_data_recording:
                    self.non_critical_flasher_timer.stop()

                # Stop the video recording and save the video (Close the video file)
                retval = self.vid_saver.close_video_file()

                self.video_recording_paused = False  # Set to not paused for next time

                self.ui.startVideoRecButton.setText("Start Recording")  # Set the button text

                if retval is True:
                    self.display_info(self.app_msg,"Video recording was stopped")
                else:
                    self.display_info(self.app_msg,"Video recording file already closed")
        else:
            self.display_info(self.app_msg,"Recording cannot be started. No video feed available")

    def start_vid_recording_button_clicked(self):
        if self.ui.startVideoRecButton.text() == "Start Recording":
            self.recording_start_stop()

        elif self.ui.startVideoRecButton.text() == "Recording...":
            self.video_recording_paused=True
            self.ui.startVideoRecButton.setText("Recording Paused")
            self.display_info(self.app_msg,"Video recording paused")

        elif self.ui.startVideoRecButton.text() == "Recording Paused":
            self.video_recording_paused=False
            self.ui.startVideoRecButton.setText("Recording...")
            self.display_info(self.app_msg,"Video recording restarted")

    def stop_vid_recording_button_clicked(self):
        if self.enable_video_recording is True:  # If recording is enabled then call the start stop function
            self.recording_start_stop()

    def training_data_recording_start_stop(self):

        if self.socket_img_connected is True:

            # Check to see the key configuration is correct before/after recording new data to the numpy file
            desired_action=self.get_key_cfg_from_meta_file(self.train_file_name,self.train_file_path)

            if desired_action is True:  # If the user has corrected for all the errors regarding key configuration
                self.enable_training_data_recording = not self.enable_training_data_recording

                if self.enable_training_data_recording is True:
                    if not self.non_critical_flasher_timer.isActive():
                        self.non_critical_flasher_timer.start()

                    # Check if the training file already exists and load it (Need to call this at least once)
                    _, self.training_frame_count=self.train_data_saver.open_training_file(self.train_file_name,
                                                                                          self.train_file_path)

                    self.ui.trainingExampleCountLabel.setText(str(self.training_frame_count))
                    self.ui.startTrainRecButton.setText("Recording...")  # Set the button text
                    self.display_info(self.app_msg,"Training data recording has started")
                else:
                    if self.non_critical_flasher_timer.isActive() and not self.enable_video_recording:
                        self.non_critical_flasher_timer.stop()

                    # Save the training data file if the recording was stopped (Close training data)
                    self.train_data_saver.close_training_data_file()

                    self.training_data_recording_paused=False

                    self.ui.trainingExampleCountLabel.setText("0")
                    self.ui.startTrainRecButton.setText("Start Recording")
                    self.display_info(self.app_msg,"Training data recording was stopped")

        else:
            self.display_info(self.app_msg, "Recording cannot be started. No video feed available")

    def start_training_data_rec_button_clicked(self):
        if self.ui.startTrainRecButton.text() == "Start Recording":
            self.training_data_recording_start_stop()

        elif self.ui.startTrainRecButton.text() == "Recording...":
            self.training_data_recording_paused=True
            self.ui.startTrainRecButton.setText("Recording paused")
            self.display_info(self.app_msg,"Training data recording paused")

        elif self.ui.startTrainRecButton.text() == "Recording paused":
            self.training_data_recording_paused=False
            self.ui.startTrainRecButton.setText("Recording...")
            self.display_info(self.app_msg,"Training data recording restarted")

    def stop_training_data_rec_button_clicked(self):
        if self.enable_training_data_recording is True:  # If recording is enabled then stop the recording
            self.training_data_recording_start_stop()

    def check_file_path_statuses(self):  # Function is used to check the status of both video and data recording path
        # This function also updates the path length to the correct full form

        # For video recording file
        self.video_path_exists, self.video_file_path = self.vid_saver.get_file_path_status()

        if not self.video_path_exists:
            self.display_info(self.app_msg,"Video recording save location did not exist")
            self.display_info(self.app_msg,"Created video save location: "+str(self.video_file_path))

        # For training data recording
        self.train_path_exists, self.train_file_path = self.train_data_saver.get_path_to_file_status()

        if not self.train_path_exists:
            self.display_info(self.app_msg,"Training data save location did not exist")
            self.display_info(self.app_msg,"Created training data save location: "+str(self.train_file_path))

    def capture_status_action(self,stat): # Function that takes action according to the stream status
        if stat == "SAO": # Image stream Active and open
            self.socket_img_connected=True  # Set connected boolean as true

            self.display_info(self.app_msg, "Video feed connection established")
            self.ui.capSocketStatusLabel.setText("Connected")

            if self.socket_data_connected is True:  # If the data connection is also connected
                self.ui.connectButton.setText("Disconnect")  # Change button to disconnect
            else:
                self.display_info(self.app_msg,"Data not connected, waiting...")

            print("Image stream is active and open")

        elif stat == "SNA":  # Stream Not Available
            self.display_black_screen()
            self.socket_img_connected=False

            # Just the thread stopper was used here and was probably not correct
            self.close_socket_routine()  # The reason is that the connect is called with out reinitialization

            self.display_info(self.app_msg, "Video feed connection lost")
            self.ui.capSocketStatusLabel.setText("Disconnected")

            if not self.socket_data_connected:  # If the data connection was also lost
                self.ui.connectButton.setText("Connect")
            else:
                self.display_info(self.app_msg,"Data connection still active")

            self.update_connection_info_bar()

            print("Image stream not available")

    def display_black_screen(self):  # Function to display a black screen
        blk_frame=self.frame_black.copy()  # Depending on the jpeg file can cause error when file selection happens
        cv2.putText(blk_frame, "VIDEO FEED NOT AVAILABLE", (50,50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,255,255), 1, cv2.LINE_AA)

        blk_pix=self.convert_frame_to_pix(blk_frame)
        self.ui.video_frame.setPixmap(blk_pix)

    # Video display settings functions
    def display_mode(self):  # Function that changes display between camera, thermal and blend mode
        if self.ui.feedTypeComboBox.currentText() == "Camera":
            self.normal_cam_enable=True
            self.thermal_cam_enable=False

            self.display_info(self.app_msg,"Normal camera feed is enabled")

        elif self.ui.feedTypeComboBox.currentText() == "Thermal Camera":
            self.normal_cam_enable=False
            self.thermal_cam_enable=True

            self.display_info(self.app_msg,"Thermal camera feed is enabled")

        elif self.ui.feedTypeComboBox.currentText() == "Camera and Thermal":
            self.normal_cam_enable=True
            self.thermal_cam_enable=True

            self.display_info(self.app_msg,"Normal and thermal camera is enabled")

    def change_display_mode_by_key(self):  # Function that gets called when key is pressed to change the display

        feed_cur_index=self.ui.feedTypeComboBox.currentIndex()

        feed_cur_index=feed_cur_index+1  # Increment the index

        if feed_cur_index > (self.ui.feedTypeComboBox.count()-1):  # Reset the index at the combobox limits
            feed_cur_index=0

        self.ui.feedTypeComboBox.setCurrentIndex(feed_cur_index)  # Set the index

    def blend_slider(self):  # Gets called when the slider value changes
        value=self.ui.blendSlider.value()

        self.norm_cam_weight=value
        self.therm_cam_weight=100-value

    def thermal_image_scale_slider(self):  # Function gets called when the scale slider changes
        self.therm_scale_val=self.ui.thermBlendScalerSlider.value()

    def thermal_image_pos_horz_slider(self):  # Function gets called when the translation slider changes
        self.therm_pos_horz=self.ui.thermPosHorzSlider.value()

    def thermal_image_pos_vert_slider(self):  # Function gets called when the translation slider changes
        self.therm_pos_vert=self.ui.thermPosVertSlider.value()

    @staticmethod
    def pad_thermal_image(img):  # Function to pad the scaled thermal image which is 160x120 or greater to 1280x720
        img_height,img_width=img.shape[:2]

        if img_width < 1280 and img_height < 720:
            top=int((720-img_height)/2)
            bottom=top

            left=int((1280-img_width)/2)
            right=left

            BLACK=[0,0,0]  # The color of the boarder

            res=cv2.copyMakeBorder(img,top,bottom,left,right,cv2.BORDER_CONSTANT,value=BLACK)
            res=cv2.resize(res,(1280,720),interpolation=cv2.INTER_LINEAR) # Resize the padded image as well

            return res

        else:
            return cv2.resize(img,(1280,720),interpolation=cv2.INTER_LINEAR)

    @staticmethod
    def scale_thermal_image(img,scaler,scale_divider):  # Scale the thermal image
        height,width=img.shape[:2]

        res = cv2.resize(img,(int(width*scaler/scale_divider),int(height*scaler/scale_divider)),
                         interpolation=cv2.INTER_LINEAR)

        return res

    @staticmethod
    def change_to_16by9_thermal_image(img):  # Function to change from 4:3 to 16:9 aspect ratio
        _,width=img.shape[:2]

        res=cv2.resize(img,(width,int(width*(9/16))),interpolation=cv2.INTER_LINEAR)

        return res

    @staticmethod
    def translate_thermal_image(img,trans_horz,trans_vert):  # Function to translate the thermal image
        rows, columns = img.shape[:2]

        TRANS_MATRIX = np.float32([[1, 0, trans_horz], [0, 1, trans_vert]])
        dst=cv2.warpAffine(img, TRANS_MATRIX, (columns, rows))

        res=cv2.resize(dst, (1280, 720), interpolation=cv2.INTER_LINEAR)  # Resize just in case an error occured

        return res

    def video_disp_set_clicked(self):  # Function to set the values
        display_mode_index=self.ui.feedTypeComboBox.currentIndex()

        self.display_info(self.app_msg,"New video display settings saved")

        self.config.set('Video_feed','feed_type',str(display_mode_index))
        self.config.set('Video_feed','camera_wgt',str(self.norm_cam_weight))
        self.config.set('Video_feed','thermal_wgt',str(self.therm_cam_weight))
        self.config.set('Video_feed','thermal_scale',str(self.therm_scale_val))
        self.config.set('Video_feed','thermal_vertical_pos',str(self.therm_pos_vert))
        self.config.set('Video_feed','thermal_horizontal_pos',str(self.therm_pos_horz))

        self.write_to_config_file()

    def video_disp_reset_clicked(self):  # Function to reset the value
        self.get_video_disp_settings_cfg(set_feed_type=False)  # The feed combo box is not reset

        self.display_info(self.app_msg,"The video display settings have been reset to previous values")

    def get_video_disp_settings_cfg(self,set_feed_type=True):  # Function to read the values for the settings
        if set_feed_type is True:
            self.ui.feedTypeComboBox.setCurrentIndex(self.config.getint('Video_feed','feed_type'))

        self.norm_cam_weight=self.config.getfloat('Video_feed','camera_wgt')
        self.therm_cam_weight=self.config.getfloat('Video_feed','thermal_wgt')

        self.ui.blendSlider.setValue(self.norm_cam_weight)

        enable_thermal_rpi=self.config.getint('Video_feed', 'enable_thermal')

        if enable_thermal_rpi == 1:
            self.ui.enableThermalCheckBox.setChecked(True)
        else:
            self.ui.enableThermalCheckBox.setChecked(False)

        self.therm_scale_val=self.config.getint('Video_feed','thermal_scale')
        self.therm_pos_vert=self.config.getint('Video_feed','thermal_vertical_pos')
        self.therm_pos_horz=self.config.getint('Video_feed','thermal_horizontal_pos')

        self.ui.thermBlendScalerSlider.setValue(self.therm_scale_val)
        self.ui.thermPosHorzSlider.setValue(self.therm_pos_horz)
        self.ui.thermPosVertSlider.setValue(self.therm_pos_vert)

        self.enable_HUD=self.config.getint('Video_feed','enable_hud')

        if self.enable_HUD == 1:
            self.ui.enableHUDCheckBox.setChecked(True)
        else:
            self.ui.enableHUDCheckBox.setChecked(False)

    def HUD_checkbox_changed(self):  # Function which gets called when checkbox changed
        if self.ui.enableHUDCheckBox.isChecked():
            self.enable_HUD=1
        else:
            self.enable_HUD=0

        self.config.set('Video_feed','enable_hud',str(self.enable_HUD))

        self.write_to_config_file()

    # Socket data connection functions
    def data_status_action(self,stat):  # Function that takes the action according to the data status
        if stat == "DAO":  # Data stream active and open
            self.socket_data_connected=True  # Set connected boolean as true

            self.display_info(self.app_msg, "Data connection established")
            self.ui.dataSocketStatusLabel.setText("Connected")

            if self.socket_img_connected is True:  # If the image stream is also connected
                self.ui.connectButton.setText("Disconnect")  # Set to disconnect
            else:
                self.display_info(self.app_msg, "Video feed not connected, waiting...")

            self.display_info(self.app_msg,"Getting configuration from robot")
            self.retrieve_configurations()  # Retrieve the settings from the RPi side

            print("Data stream active and open")

        elif stat == "DNA":  # Data stream Not Available
            self.socket_data_connected=False

            # Just the thread stopper was used here and was probably not correct
            self.close_socket_routine()  # The reason is that the connect is called with out reinitialization

            self.display_info(self.app_msg, "Data connection lost")
            self.ui.dataSocketStatusLabel.setText("Disconnected")

            if not self.socket_img_connected:  # If the image stream is also not connected
                self.ui.connectButton.setText("Connect")  # Set to connect
            else:
                self.display_info(self.app_msg, "Video feed connection still active")

            self.update_connection_info_bar()

            print("Data stream not available")

    def get_socket_stream_data(self, data_bytes):  # Function to process the data received over the socket
        data_str=data_bytes.decode('utf-8')
        self.handle_socket_data(data_str)  # Call the data handler

    def handle_socket_data(self,data):

        parsed_data=data.split(",")

        header=parsed_data[0]  # Get the header

        if header == self.socket_receive_header_dict['GAS']:
            self.eCO2=int(parsed_data[1])
            self.TVOC=int(parsed_data[2])

            self.check_gas_sensor_readings()  # Alarm is set if threshold is reached

            self.update_gas_readings_labels(self.eCO2,self.TVOC)

        elif header == self.socket_receive_header_dict['PARTICLE']:
            self.red=int(parsed_data[1])
            self.green=int(parsed_data[2])
            self.ir=int(parsed_data[3])

            self.check_particle_sensor_readings()  # Alarm is set if threshold is reached

            self.update_particle_readings_labels(self.red,self.green,self.ir)

        elif header == self.socket_receive_header_dict['ENVIRONMENTAL']:
            self.pressure=float(parsed_data[1])
            self.temperature=float(parsed_data[2])
            self.humidity=float(parsed_data[3])

            self.update_environment_readings_labels(self.temperature,self.pressure,self.humidity)

        elif header == self.socket_receive_header_dict['WIFI']:
            self.wifi_link_quality=float(parsed_data[1])
            self.wifi_signal_level=int(parsed_data[2])

            self.wifi_link_quality=round(self.wifi_link_quality,2)

            self.update_wifi_status()

        elif header == self.socket_receive_header_dict['SOCK_SET_ACK']:

            # Get the acknowledgement code
            ack_code=int(parsed_data[1])

            if ack_code == self.ack_code_dict['GAS_THR']:
                self.display_info(self.bot_msg, "The gas thresholds were applied successfully")

            elif ack_code == self.ack_code_dict['PAR_THR']:
                self.display_info(self.bot_msg, "The particle thresholds were applied successfully")

            elif ack_code == self.ack_code_dict['SMS_SET']:
                self.display_info(self.bot_msg, "The SMS setting were applied successfully")

            elif ack_code == self.ack_code_dict['CON_SET']:
                self.display_info(self.bot_msg, "The connection settings were applied successfully")

            elif ack_code == self.ack_code_dict['VID_SET']:
                self.display_info(self.bot_msg, "The video settings were applied successfully")

            elif ack_code == self.ack_code_dict['RST_ALM']:
                self.display_info(self.bot_msg, "The alarm was reset successfully")

        elif header == self.socket_receive_header_dict['GAS_THR_RET']:
            self.eCO2_thresh=int(parsed_data[1])
            self.TVOC_thresh=int(parsed_data[2])

            self.ui.eCO2ThresholdSpinBox.setValue(self.eCO2_thresh)
            self.ui.tvocThresholdSpinBox.setValue(self.TVOC_thresh)

            self.config.set('Gas_thresh', 'eco2', str(self.eCO2_thresh))
            self.config.set('Gas_thresh', 'tvoc', str(self.TVOC_thresh))

            self.write_to_config_file()

            self.display_info(self.app_msg,"Gas threshold settings updated")

            self.updated_settings_dict['GAS_THR']=True

        elif header == self.socket_receive_header_dict['PAR_THR_RET']:
            self.Red_thresh=int(parsed_data[1])
            self.Green_thresh=int(parsed_data[2])
            self.IR_thresh=int(parsed_data[3])

            self.ui.redThresholdSpinBox.setValue(self.Red_thresh)
            self.ui.greenThresholdSpinBox.setValue(self.Green_thresh)
            self.ui.irThresholdSpinBox.setValue(self.IR_thresh)

            self.config.set('Particle_thresh', 'red', str(self.Red_thresh))
            self.config.set('Particle_thresh', 'green', str(self.Green_thresh))
            self.config.set('Particle_thresh', 'ir', str(self.IR_thresh))

            self.write_to_config_file()

            self.display_info(self.app_msg,"Particle threshold settings updated")

            self.updated_settings_dict['PAR_THR']=True

        elif header == self.socket_receive_header_dict['SMS_RET']:
            self.sms_number=parsed_data[1]
            self.sms_enabled=int(parsed_data[2])

            self.ui.smsNumberLineEdit.setText(str(self.sms_number))

            if self.sms_enabled == 1:
                self.ui.smsAlertEnableCheckBox.setChecked(True)
            elif self.sms_enabled == 0:
                self.ui.smsAlertEnableCheckBox.setChecked(False)

            self.config.set('SMS_alert', 'alert_number', str(self.sms_number))
            self.config.set('SMS_alert', 'alert_enable', str(self.sms_enabled))

            self.write_to_config_file()

            self.display_info(self.app_msg,"SMS settings updated")

            self.updated_settings_dict['SMS']=True

        elif header == self.socket_receive_header_dict['CAM_RET']:
            vid_fps=int(parsed_data[1])
            vid_color_index=int(parsed_data[2])
            vid_res_index=int(parsed_data[3])
            enable_thermal_rpi=int(parsed_data[4])

            self.ui.videoResComboBox.setCurrentIndex(vid_res_index)
            self.ui.frameRateSpinBox.setValue(vid_fps)
            self.ui.capColorComboBox.setCurrentIndex(vid_color_index)

            if enable_thermal_rpi == 1:
                self.ui.enableThermalCheckBox.setChecked(True)
            elif enable_thermal_rpi == 0:
                self.ui.enableThermalCheckBox.setChecked(False)

            self.config.set('Video_feed', 'resolution', str(vid_res_index))
            self.config.set('Video_feed', 'fps', str(vid_fps))
            self.config.set('Video_feed', 'color', str(vid_color_index))
            self.config.set('Video_feed', 'enable_thermal', str(enable_thermal_rpi))

            self.write_to_config_file()

            self.display_info(self.app_msg,"Video feed settings updated")

            self.updated_settings_dict['CAM_SET']=True

        elif header == self.socket_receive_header_dict['MCU_STAT']:
            status=int(parsed_data[1])

            if status == 1:  # Means microcontroller is ready
                self.micro_controller_rdy=True
                self.display_info(self.app_msg,"Microcontroller is ready")

            elif status == 0:  # Means microcontroller is not ready
                self.micro_controller_rdy=False
                self.display_info(self.app_msg,"Microcontroller is not ready")

        else:
            return

    def send_socket_commands(self, data_str):  # Function to send commands via socket
        if self.socket_data_connected:  # If the socket for data is connected then only send
            self.soc_data.send_via_socket(data_str)  # send the data

        elif (time.time() - self.prev_msg_disp_time) > self.msg_delay:  # Display repeat with a delay
            self.display_info(self.app_msg,"Data socket not connected! Please connect to a client")
            print("Data socket not connected!")
            self.prev_msg_disp_time=time.time()

    # Function to update the sensor values
    def update_gas_readings_labels(self,eco2,tvoc):  # Function for gas sensor readings
        self.ui.co2Label.setText(str(eco2))
        self.ui.tvocLabel.setText(str(tvoc))

    def update_particle_readings_labels(self,red,green,ir):  # Function for particle sensor readings
        self.ui.redLabel.setText(str(red))
        self.ui.greenLabel.setText(str(green))
        self.ui.irLabel.setText(str(ir))

    def update_environment_readings_labels(self,temp,pres,hum):  # Function for environmental readings
        self.ui.temperatureLabel.setText(str(temp))
        self.ui.pressureLabel.setText(str(pres))
        self.ui.humidityLabel.setText(str(hum))

    # Functions to check the sensor readings against the threshold (Handle data function)
    def check_gas_sensor_readings(self):
        if self.eCO2 >= self.eCO2_thresh or self.TVOC >= self.TVOC_thresh:
            self.gas_threshold_reached=True

            self.set_max_for_gas_sensor()

            if not self.warning_flasher_timer.isActive():  # If timer is not active
                self.warning_flasher_timer.start()

    def set_max_for_gas_sensor(self):
        max_val_changed=False

        if self.eCO2 >= self.cur_max_eCO2:
            self.cur_max_eCO2=self.eCO2
            max_val_changed=True

        if self.TVOC >= self.cur_max_TVOC:
            self.cur_max_TVOC=self.TVOC
            max_val_changed=True

        if max_val_changed is True:
            self.display_info(self.app_msg,"Gas sensor threshold reached")
            self.display_info(self.app_msg,"Max gas sensor readings")
            self.display_info(self.app_msg,"Maximum eCO2 detected: "+str(self.cur_max_eCO2)+" ppm")
            self.display_info(self.app_msg,"Maximum TVOC detected: "+str(self.cur_max_TVOC)+" ppb")

    def check_particle_sensor_readings(self):
        if self.red >= self.Red_thresh and self.green >= self.Green_thresh and self.ir >= self.IR_thresh:
            self.particle_threshold_reached=True

            self.set_max_for_particle_sensor()

            if not self.warning_flasher_timer.isActive():  # If timer is not active
                self.warning_flasher_timer.start()

    def set_max_for_particle_sensor(self):
        max_val_changed = False

        if self.red >= self.cur_max_Red and self.green >= self.cur_max_Green and self.ir >= self.cur_max_IR:
            self.cur_max_Red = self.red
            self.cur_max_Green = self.green
            self.cur_max_IR = self.ir
            max_val_changed = True

        if max_val_changed is True:
            self.display_info(self.app_msg, "Particle sensor threshold reached")
            self.display_info(self.app_msg, "Max particle sensor readings")
            self.display_info(self.app_msg, "Maximum Red: " + str(self.cur_max_Red))
            self.display_info(self.app_msg, "Maximum Green: " + str(self.cur_max_Green))
            self.display_info(self.app_msg, "Maximum IR: " + str(self.cur_max_IR))

    def reset_alarm(self):  # Function to reset the alarm
        if self.gas_threshold_reached is True:
            self.display_info(self.app_msg,"Gas warning alarm reset")

        if self.particle_threshold_reached is True:
            self.display_info(self.app_msg,"Particle warning alarm reset")

        # Reset all the variables related to the alarm
        self.gas_threshold_reached=False
        self.particle_threshold_reached=False

        self.cur_max_eCO2=0
        self.cur_max_TVOC=0

        self.cur_max_Red=0
        self.cur_max_Green=0
        self.cur_max_IR=0

        if self.warning_flasher_timer.isActive():
            self.warning_flasher_timer.stop()

        # A command to reset the alarm in the microcontroller should be sent as well
        cmd=self.format_command(self.socket_send_header_dict['RESET_ALARM'],[])

        self.send_socket_commands(cmd)

    # Video feed settings
    def video_feed_config_set_clicked(self):  # Get the user specified video feed settings and send the command to set

        vid_res=self.ui.videoResComboBox.currentText()
        vid_res_index=self.ui.videoResComboBox.currentIndex()

        vid_fps=self.ui.frameRateSpinBox.value()

        vid_color=self.ui.capColorComboBox.currentText()
        vid_color_index=self.ui.capColorComboBox.currentIndex()

        enable_thermal_rpi = 0
        disp_str = ""

        if self.ui.enableThermalCheckBox.isChecked():
            enable_thermal_rpi=1
            disp_str="Enabled"
        else:
            enable_thermal_rpi=0
            disp_str="Disabled"

        self.display_info(self.app_msg,"New video feed settings will be applied")
        self.display_info(self.app_msg,"Capture resolution: "+str(vid_res))
        self.display_info(self.app_msg,"Capture FPS: "+str(vid_fps))
        self.display_info(self.app_msg,"Capture color: "+str(vid_color))
        self.display_info(self.app_msg, "Raspberry Pi thermal camera: " + disp_str)

        self.config.set('Video_feed','resolution',str(vid_res_index))
        self.config.set('Video_feed','fps',str(vid_fps))
        self.config.set('Video_feed','color',str(vid_color_index))
        self.config.set('Video_feed','enable_thermal',str(enable_thermal_rpi))

        self.write_to_config_file()

        # Command to set the capture resolution fps and color is sent to RPi
        parse_res=vid_res.split("x")
        width=parse_res[0]
        height=parse_res[1]

        cmd=self.format_command(self.socket_send_header_dict['CAM_SET'],[width,height,vid_fps,
                                                                        vid_color_index,vid_res_index,
                                                                         enable_thermal_rpi])

        self.send_socket_commands(cmd)

    def video_feed_config_reset_clicked(self):
        self.get_connection_settings_cfg()

        self.display_info(self.app_msg,"The video feed settings have been reset to previous values")

    def get_video_feed_setting_cfg(self):
        self.ui.videoResComboBox.setCurrentIndex(self.config.getint('Video_feed', 'resolution'))
        self.ui.frameRateSpinBox.setValue(self.config.getint('Video_feed', 'fps'))
        self.ui.capColorComboBox.setCurrentIndex(self.config.getint('Video_feed', 'color'))

        self.enable_thermal_video_checkbox_changed()

    def enable_thermal_video_checkbox_changed(self):

        enb_disb=self.ui.enableThermalCheckBox.isChecked()

        enb_disb=not enb_disb  # Invert (As checked means disabled so false must be passed)

        # Here false is needed to disable
        self.ui.blendSlider.setDisabled(enb_disb)
        self.ui.thermBlendScalerSlider.setDisabled(enb_disb)
        self.ui.thermPosVertSlider.setDisabled(enb_disb)
        self.ui.thermPosHorzSlider.setDisabled(enb_disb)

        # Here true is needed to disable
        self.ui.feedTypeComboBox.setCurrentIndex(0)  # Set to normal camera mode
        self.ui.feedTypeComboBox.model().item(1).setEnabled(not enb_disb)
        self.ui.feedTypeComboBox.model().item(2).setEnabled(not enb_disb)

    # Video recording settings functions
    def video_recording_config_set_clicked(self):  # Function to set new video recording settings
        self.video_file_name=self.ui.videoFileNameLineEdit.text()
        self.video_file_path=self.ui.videoFilePathLineEdit.text()
        self.rec_res_index=self.ui.videoRecResolutionComboBox.currentIndex()
        vid_rec_res=self.ui.videoRecResolutionComboBox.currentText()
        self.rec_fps=self.ui.videoRecordingFPSSpinBox.value()

        res_enable_string=""
        # Check to see if the use capture resolution is enabled
        if self.ui.useCaptureResCheckBox.isChecked():
            self.use_cap_resolution=1
            vid_rec_res=self.ui.videoResComboBox.currentText()  # Get the current video capture resolution
            res_enable_string="Enabled"
        else:
            self.use_cap_resolution=0
            res_enable_string="Disabled"

        fps_enable_string=""
        # Check to see if the use capture FPS is enabled
        if self.ui.useCaptureFPSCheckBox.isChecked():
            self.use_cap_fps=1
            self.rec_fps=self.ui.frameRateSpinBox.value()  # Get the capture FPS
            fps_enable_string="Enabled"
        else:
            self.use_cap_fps=0
            fps_enable_string="Disabled"

        # Set the recording width and height
        parse_vid_rec_res=vid_rec_res.split("x")
        self.rec_width=int(parse_vid_rec_res[0])
        self.rec_height=int(parse_vid_rec_res[1])

        self.display_info(self.app_msg,"New video recording settings applied")
        self.display_info(self.app_msg,"Video recording file name: "+str(self.video_file_name))
        self.display_info(self.app_msg,"Video recording file path: "+str(self.video_file_path))
        self.display_info(self.app_msg,"Video recording resolution: "+str(vid_rec_res))
        self.display_info(self.app_msg,"Video recording FPS: "+str(self.rec_fps))
        self.display_info(self.app_msg,"Use capture resolution: "+str(res_enable_string))
        self.display_info(self.app_msg,"USe capture FPS: "+str(fps_enable_string))

        # The relative path is stored instead of the absolute path
        rel_path_save=os.path.relpath(self.video_file_path, start=os.path.expanduser('~'))

        self.config.set('Video_rec','vid_file_name',str(self.video_file_name))
        self.config.set('Video_rec','vid_file_path',str(rel_path_save))
        self.config.set('Video_rec','vid_rec_res',str(self.rec_res_index))
        self.config.set('Video_rec','vid_rec_fps',str(self.rec_fps))
        self.config.set('Video_rec','use_cap_res',str(self.use_cap_resolution))
        self.config.set('Video_rec','use_cap_fps',str(self.use_cap_fps))

        # Applying new properties (The video recording will be automatically stopped here if was enabled)
        # The video file will be closed here as well
        self.vid_saver.change_video_save_prop(name=self.video_file_name,
                                              path=self.video_file_path,
                                              width=self.rec_width,
                                              height=self.rec_height,
                                              fps=self.rec_fps)

        if self.enable_video_recording is True:
            self.recording_start_stop()
            self.display_info(self.app_msg,"Video recording was stopped as recording properties was changed")

        self.write_to_config_file()

    def video_recording_config_reset_clicked(self):  # Function to reset the video recording settings
        self.get_video_recording_setting_cfg()

        self.display_info(self.app_msg,"The video recording settings have been reset to previous values")

    def get_video_recording_setting_cfg(self):  # Function to get the video recording configuration

        self.video_file_name=self.config.get('Video_rec', 'vid_file_name')
        self.video_file_path=os.path.expanduser(self.config.get('Video_rec', 'vid_file_path'))
        self.rec_res_index=self.config.getint('Video_rec', 'vid_rec_res')
        self.rec_fps=self.config.getint('Video_rec', 'vid_rec_fps')
        self.use_cap_resolution=self.config.getint('Video_rec','use_cap_res')
        self.use_cap_fps=self.config.getint('Video_rec','use_cap_fps')

        self.ui.videoFileNameLineEdit.setText(self.video_file_name)
        self.ui.videoFilePathLineEdit.setText(self.video_file_path)

        self.ui.videoRecResolutionComboBox.setCurrentIndex(self.rec_res_index)
        vid_rec_res = self.ui.videoRecResolutionComboBox.currentText()

        if self.use_cap_resolution == 1:
            vid_rec_res=self.ui.videoResComboBox.currentText()  # Get the current video capture resolution
            self.ui.useCaptureResCheckBox.setChecked(True)
        else:
            self.ui.useCaptureResCheckBox.setChecked(False)

        if self.use_cap_fps == 1:
            self.rec_fps = self.ui.frameRateSpinBox.value()  # Get the capture FPS
            self.ui.useCaptureFPSCheckBox.setChecked(True)
        else:
            self.ui.useCaptureFPSCheckBox.setChecked(False)

        parse_vid_rec_res = vid_rec_res.split("x")
        self.rec_width = int(parse_vid_rec_res[0])
        self.rec_height = int(parse_vid_rec_res[1])

        self.ui.videoRecordingFPSSpinBox.setValue(self.rec_fps)

    def open_video_recording_file_location(self):  # Function to open the video recording file location
        try:
            os.startfile(self.video_file_path)
        except FileNotFoundError:
            print("File does not exist")

    def browse_for_video_recording_file_path(self):
        path_to_vid_rec_file=QtGui.QFileDialog.getExistingDirectory(parent=None,
                                                                    caption="Select video recording save folder",
                                                                    directory=self.video_file_path,
                                                                    options=QtGui.QFileDialog.ShowDirsOnly)

        if not path_to_vid_rec_file == '':  # If path is not empty
            self.ui.videoFilePathLineEdit.setText(str(path_to_vid_rec_file))  # Set string to the selected folder path

    def disable_rec_vid_res_fps(self):
        if self.ui.useCaptureResCheckBox.isChecked():
            self.ui.videoRecResolutionComboBox.setEnabled(False)
        else:
            self.ui.videoRecResolutionComboBox.setEnabled(True)

        if self.ui.useCaptureFPSCheckBox.isChecked():
            self.ui.videoRecordingFPSSpinBox.setEnabled(False)
        else:
            self.ui.videoRecordingFPSSpinBox.setEnabled(True)

    # Training data recording settings
    def training_recording_config_set_clicked(self):
        self.train_file_name = self.ui.trainingFileNameLineEdit.text()
        self.train_file_path = self.ui.trainingFilePathLineEdit.text()
        self.train_frame_width = self.ui.imgWidthSpinBox.value()
        self.train_frame_height = self.ui.imgHeightSpinBox.value()
        self.save_per_every = self.ui.autoSaveSampleSpinBox.value()

        self.display_info(self.app_msg,"New training data recording settings applied")
        self.display_info(self.app_msg,"Training data recording file name: "+self.train_file_name)
        self.display_info(self.app_msg,"Training data recording file path: "+self.train_file_path)
        self.display_info(self.app_msg,"Training frame resolution: "+
                          str(self.train_frame_width)+"x"+str(self.train_frame_height))
        self.display_info(self.app_msg,"Automatic save per every: "+str(self.save_per_every)+" sample(s)")

        # The relative path is stored instead of the absolute path
        rel_path_save = os.path.relpath(self.train_file_path, start=os.path.expanduser('~'))

        self.config.set('Training_rec', 'train_file_name',str(self.train_file_name))
        self.config.set('Training_rec', 'train_file_path',str(rel_path_save))
        self.config.set('Training_rec', 'train_frame_width',str(self.train_frame_width))
        self.config.set('Training_rec', 'train_frame_height',str(self.train_frame_height))
        self.config.set('Training_rec', 'save_per_every',str(self.save_per_every))

        self.train_data_saver.change_train_save_prop(name=self.train_file_name,
                                                     path=self.train_file_path,
                                                     saves_ps=self.save_per_every)

        if self.enable_training_data_recording is True:
            self.training_data_recording_start_stop()
            self.display_info(self.app_msg,"Training data recording was stopped as properties were changed")

        self.write_to_config_file()

    def training_recording_config_reset_clicked(self):
        self.get_training_recording_setting_cfg()

        self.display_info(self.app_msg,"The training data recording settings have been reset to previous values")

    def get_training_recording_setting_cfg(self):
        self.train_file_name=self.config.get('Training_rec','train_file_name')
        self.train_file_path=os.path.expanduser(self.config.get('Training_rec','train_file_path'))
        self.train_frame_width=self.config.getint('Training_rec','train_frame_width')
        self.train_frame_height=self.config.getint('Training_rec','train_frame_height')
        self.save_per_every=self.config.getint('Training_rec','save_per_every')

        self.ui.trainingFileNameLineEdit.setText(self.train_file_name)
        self.ui.trainingFilePathLineEdit.setText(self.train_file_path)
        self.ui.imgWidthSpinBox.setValue(self.train_frame_width)
        self.ui.imgHeightSpinBox.setValue(self.train_frame_height)
        self.ui.autoSaveSampleSpinBox.setValue(self.save_per_every)

    def open_training_data_file_location_and_select(self):  # Function to open the training file and select the file
        selected_train_file_name=QtGui.QFileDialog.getOpenFileName(parent=None,
                                                                   caption="Select a training data set numpy file",
                                                                   directory=self.train_file_path,
                                                                   filter="Numpy files(*.npy)",)

        # Using QFileInfo class to get file name
        new_train_file_name=QtCore.QFileInfo(selected_train_file_name).fileName()

        if not new_train_file_name == '':
            self.ui.trainingFileNameLineEdit.setText(str(new_train_file_name))

    def browse_for_training_recording_file_path(self):
        path_to_train_rec_file=QtGui.QFileDialog.getExistingDirectory(parent=None,
                                                                      caption="Select training data recording "
                                                                              "save folder",
                                                                      directory=self.train_file_path,
                                                                      options=QtGui.QFileDialog.ShowDirsOnly)

        if not path_to_train_rec_file == '':  # If path is not empty
            self.ui.trainingFilePathLineEdit.setText(str(path_to_train_rec_file))

    # Disabled keys and key label order section under training data recording section
    def disabled_key_config_set_clicked(self):
        self.disabled_button_press_dict['FORWARD'] = self.ui.forwardCheckBox.isChecked()
        self.disabled_button_press_dict['BACKWARD'] = self.ui.backwardCheckBox.isChecked()
        self.disabled_button_press_dict['LEFT'] = self.ui.leftCheckBox.isChecked()
        self.disabled_button_press_dict['RIGHT'] = self.ui.rightCheckBox.isChecked()
        self.disabled_button_press_dict['LEFT_TURN_FORWARD'] = self.ui.leftTurnForwardCheckBox.isChecked()
        self.disabled_button_press_dict['RIGHT_TURN_FORWARD'] = self.ui.rightTurnForwardCheckBox.isChecked()
        self.disabled_button_press_dict['LEFT_TURN_BACKWARD'] = self.ui.leftTurnBackwardCheckBox.isChecked()
        self.disabled_button_press_dict['RIGHT_TURN_BACKWARD'] = self.ui.rightTurnBackwardCheckBox.isChecked()
        self.disabled_button_press_dict['ACCELERATION'] = self.ui.accelerationCheckBox.isChecked()
        self.disabled_button_press_dict['BRAKE'] = self.ui.brakeCheckBox.isChecked()

        self.move_button_press_order_dict['FORWARD'] = self.ui.forwardOrderSpinBox.value()
        self.move_button_press_order_dict['BACKWARD'] = self.ui.backwardOrderSpinBox.value()
        self.move_button_press_order_dict['LEFT'] = self.ui.leftTurnOrderSpinBox.value()
        self.move_button_press_order_dict['RIGHT'] = self.ui.rightTurnOrderSpinBox.value()
        self.move_button_press_order_dict['LEFT_TURN_FORWARD'] = self.ui.l_turn_fOrderSpinBox.value()
        self.move_button_press_order_dict['RIGHT_TURN_FORWARD'] = self.ui.r_turn_fOrderSpinBox.value()
        self.move_button_press_order_dict['LEFT_TURN_BACKWARD'] = self.ui.l_turn_bOrderSpinBox.value()
        self.move_button_press_order_dict['RIGHT_TURN_BACKWARD'] = self.ui.r_turn_bOrderSpinBox.value()
        self.move_button_press_order_dict['ACCELERATION'] = self.ui.acclOrderSpinBox.value()
        self.move_button_press_order_dict['BRAKE'] = self.ui.brakeOrderSpinBox.value()

        self.disable_send_command=self.ui.disableSendCmdCheckBox.isChecked()

        # Check the key order
        retval=self.check_for_same_key_orders(inp_disb_dict=self.disabled_button_press_dict,
                                              inp_order_dict=self.move_button_press_order_dict)

        # If redundant key order values are found
        if retval is False:
            self.display_info(self.app_msg,"Keys are sharing the same order, please use unique values")

            self.show_key_order_warning()

            # Redistribute the key order value
            self.redistribute_key_order(inp_disb_dict=self.disabled_button_press_dict,
                                        inp_order_dict=self.move_button_press_order_dict)

        # Check if the changes applied are good for use or not
        desired_action=self.get_key_cfg_from_meta_file(file_name=self.train_file_name, file_path=self.train_file_path)

        if desired_action is True:  # If a true is returned from the above function
            self.config.set('Ctrl_key_cfg', 'forward', str(int(self.disabled_button_press_dict['FORWARD'])))
            self.config.set('Ctrl_key_cfg', 'backward', str(int(self.disabled_button_press_dict['BACKWARD'])))
            self.config.set('Ctrl_key_cfg', 'left', str(int(self.disabled_button_press_dict['LEFT'])))
            self.config.set('Ctrl_key_cfg', 'right', str(int(self.disabled_button_press_dict['RIGHT'])))
            self.config.set('Ctrl_key_cfg', 'l_turn_f', str(int(self.disabled_button_press_dict['LEFT_TURN_FORWARD'])))
            self.config.set('Ctrl_key_cfg', 'r_turn_f', str(int(self.disabled_button_press_dict['RIGHT_TURN_FORWARD'])))
            self.config.set('Ctrl_key_cfg', 'l_turn_b', str(int(self.disabled_button_press_dict['LEFT_TURN_BACKWARD'])))
            self.config.set('Ctrl_key_cfg', 'r_turn_b',str(int(self.disabled_button_press_dict['RIGHT_TURN_BACKWARD'])))
            self.config.set('Ctrl_key_cfg', 'accel', str(int(self.disabled_button_press_dict['ACCELERATION'])))
            self.config.set('Ctrl_key_cfg', 'brake', str(int(self.disabled_button_press_dict['BRAKE'])))

            self.config.set('Ctrl_key_cfg', 'forward_ord', str(self.move_button_press_order_dict['FORWARD']))
            self.config.set('Ctrl_key_cfg', 'backward_ord', str(self.move_button_press_order_dict['BACKWARD']))
            self.config.set('Ctrl_key_cfg', 'left_ord', str(self.move_button_press_order_dict['LEFT']))
            self.config.set('Ctrl_key_cfg', 'right_ord', str(self.move_button_press_order_dict['RIGHT']))
            self.config.set('Ctrl_key_cfg', 'l_turn_f_ord', str(self.move_button_press_order_dict['LEFT_TURN_FORWARD']))
            self.config.set('Ctrl_key_cfg', 'r_turn_f_ord', str(self.move_button_press_order_dict['RIGHT_TURN_FORWARD']))
            self.config.set('Ctrl_key_cfg', 'l_turn_b_ord', str(self.move_button_press_order_dict['LEFT_TURN_BACKWARD']))
            self.config.set('Ctrl_key_cfg', 'r_turn_b_ord', str(self.move_button_press_order_dict['RIGHT_TURN_BACKWARD']))
            self.config.set('Ctrl_key_cfg', 'accel_ord', str(self.move_button_press_order_dict['ACCELERATION']))
            self.config.set('Ctrl_key_cfg', 'brake_ord', str(self.move_button_press_order_dict['BRAKE']))

            self.config.set('Ctrl_key_cfg','disable_send_command',str(int(self.disable_send_command)))

            # Update the list length only if everything check out with the meta file
            self.keys_comb_len, self.default_move_list = self.generate_allowed_key_list(inp_disb_dict=self.disabled_button_press_dict)

            self.update_copies_of_key_order_disable_dict()
            self.disable_key_order_spin_box(redistribute=False)  # The max spin box values are updated here

            self.display_info(self.app_msg,"New disabled keys and output label order settings applied")

            self.write_to_config_file()

        if self.enable_training_data_recording is True:
            self.training_data_recording_start_stop()
            self.display_info(self.app_msg,"Training data recording was stopped as key settings were changed")
            self.display_info(self.app_msg,"New training data set file should be used to make use of new key settings")

    def disabled_key_config_reset_clicked(self):
        self.get_disabled_key_settings_cfg()

        self.display_info(self.app_msg,"Disabled key and order settings have been reset to previous values")

    def get_disabled_key_settings_cfg(self):  # Function to get configuration of disabled keys
        self.disabled_button_press_dict['FORWARD']=self.config.getboolean('Ctrl_key_cfg','forward')
        self.disabled_button_press_dict['BACKWARD']=self.config.getboolean('Ctrl_key_cfg','backward')
        self.disabled_button_press_dict['LEFT']=self.config.getboolean('Ctrl_key_cfg','left')
        self.disabled_button_press_dict['RIGHT']=self.config.getboolean('Ctrl_key_cfg','right')
        self.disabled_button_press_dict['LEFT_TURN_FORWARD']=self.config.getboolean('Ctrl_key_cfg','l_turn_f')
        self.disabled_button_press_dict['RIGHT_TURN_FORWARD']=self.config.getboolean('Ctrl_key_cfg','r_turn_f')
        self.disabled_button_press_dict['LEFT_TURN_BACKWARD']=self.config.getboolean('Ctrl_key_cfg','l_turn_b')
        self.disabled_button_press_dict['RIGHT_TURN_BACKWARD']=self.config.getboolean('Ctrl_key_cfg','r_turn_b')
        self.disabled_button_press_dict['ACCELERATION']=self.config.getboolean('Ctrl_key_cfg','accel')
        self.disabled_button_press_dict['BRAKE']=self.config.getboolean('Ctrl_key_cfg','brake')

        self.move_button_press_order_dict['FORWARD']=self.config.getint('Ctrl_key_cfg','forward_ord')
        self.move_button_press_order_dict['BACKWARD']=self.config.getint('Ctrl_key_cfg','backward_ord')
        self.move_button_press_order_dict['LEFT']=self.config.getint('Ctrl_key_cfg','left_ord')
        self.move_button_press_order_dict['RIGHT']=self.config.getint('Ctrl_key_cfg','right_ord')
        self.move_button_press_order_dict['LEFT_TURN_FORWARD']=self.config.getint('Ctrl_key_cfg','l_turn_f_ord')
        self.move_button_press_order_dict['RIGHT_TURN_FORWARD']=self.config.getint('Ctrl_key_cfg','r_turn_f_ord')
        self.move_button_press_order_dict['LEFT_TURN_BACKWARD']=self.config.getint('Ctrl_key_cfg','l_turn_b_ord')
        self.move_button_press_order_dict['RIGHT_TURN_BACKWARD']=self.config.getint('Ctrl_key_cfg','r_turn_b_ord')
        self.move_button_press_order_dict['ACCELERATION']=self.config.getint('Ctrl_key_cfg','accel_ord')
        self.move_button_press_order_dict['BRAKE']=self.config.getint('Ctrl_key_cfg','brake_ord')

        self.disable_send_command=self.config.getboolean('Ctrl_key_cfg','disable_send_command')

        # Update the list length and the move key press list
        self.keys_comb_len,self.default_move_list=self.generate_allowed_key_list(inp_disb_dict=self.disabled_button_press_dict)

        # Disable the check box signals
        self.ui.forwardCheckBox.blockSignals(True)
        self.ui.backwardCheckBox.blockSignals(True)
        self.ui.leftCheckBox.blockSignals(True)
        self.ui.rightCheckBox.blockSignals(True)
        self.ui.leftTurnForwardCheckBox.blockSignals(True)
        self.ui.rightTurnForwardCheckBox.blockSignals(True)
        self.ui.leftTurnBackwardCheckBox.blockSignals(True)
        self.ui.rightTurnBackwardCheckBox.blockSignals(True)
        self.ui.accelerationCheckBox.blockSignals(True)
        self.ui.brakeCheckBox.blockSignals(True)

        self.ui.forwardCheckBox.setChecked(self.disabled_button_press_dict['FORWARD'])
        self.ui.backwardCheckBox.setChecked(self.disabled_button_press_dict['BACKWARD'])
        self.ui.leftCheckBox.setChecked(self.disabled_button_press_dict['LEFT'])
        self.ui.rightCheckBox.setChecked(self.disabled_button_press_dict['RIGHT'])
        self.ui.leftTurnForwardCheckBox.setChecked(self.disabled_button_press_dict['LEFT_TURN_FORWARD'])
        self.ui.rightTurnForwardCheckBox.setChecked(self.disabled_button_press_dict['RIGHT_TURN_FORWARD'])
        self.ui.leftTurnBackwardCheckBox.setChecked(self.disabled_button_press_dict['LEFT_TURN_BACKWARD'])
        self.ui.rightTurnBackwardCheckBox.setChecked(self.disabled_button_press_dict['RIGHT_TURN_BACKWARD'])
        self.ui.accelerationCheckBox.setChecked(self.disabled_button_press_dict['ACCELERATION'])
        self.ui.brakeCheckBox.setChecked(self.disabled_button_press_dict['BRAKE'])

        self.update_copies_of_key_order_disable_dict()
        self.disable_key_order_spin_box(redistribute=False)  # Here the maximum value for the spin box is also updated

        # Enable the check box signals
        self.ui.forwardCheckBox.blockSignals(False)
        self.ui.backwardCheckBox.blockSignals(False)
        self.ui.leftCheckBox.blockSignals(False)
        self.ui.rightCheckBox.blockSignals(False)
        self.ui.leftTurnForwardCheckBox.blockSignals(False)
        self.ui.rightTurnForwardCheckBox.blockSignals(False)
        self.ui.leftTurnBackwardCheckBox.blockSignals(False)
        self.ui.rightTurnBackwardCheckBox.blockSignals(False)
        self.ui.accelerationCheckBox.blockSignals(False)
        self.ui.brakeCheckBox.blockSignals(False)

        self.ui.forwardOrderSpinBox.setValue(self.move_button_press_order_dict['FORWARD'])
        self.ui.backwardOrderSpinBox.setValue(self.move_button_press_order_dict['BACKWARD'])
        self.ui.leftTurnOrderSpinBox.setValue(self.move_button_press_order_dict['LEFT'])
        self.ui.rightTurnOrderSpinBox.setValue(self.move_button_press_order_dict['RIGHT'])
        self.ui.l_turn_fOrderSpinBox.setValue(self.move_button_press_order_dict['LEFT_TURN_FORWARD'])
        self.ui.r_turn_fOrderSpinBox.setValue(self.move_button_press_order_dict['RIGHT_TURN_FORWARD'])
        self.ui.l_turn_bOrderSpinBox.setValue(self.move_button_press_order_dict['LEFT_TURN_BACKWARD'])
        self.ui.r_turn_bOrderSpinBox.setValue(self.move_button_press_order_dict['RIGHT_TURN_BACKWARD'])
        self.ui.acclOrderSpinBox.setValue(self.move_button_press_order_dict['ACCELERATION'])
        self.ui.brakeOrderSpinBox.setValue(self.move_button_press_order_dict['BRAKE'])

        self.ui.disableSendCmdCheckBox.setChecked(self.disable_send_command)

    def disable_key_order_spin_box(self,redistribute=True):  # Function is connected to check box state changed signal
        if self.ui.forwardCheckBox.isChecked():
            self.updated_disabled_key_dict['FORWARD']=True
            self.ui.forwardOrderSpinBox.setEnabled(False)
        else:
            self.updated_disabled_key_dict['FORWARD']=False
            self.ui.forwardOrderSpinBox.setEnabled(True)

        if self.ui.backwardCheckBox.isChecked():
            self.updated_disabled_key_dict['BACKWARD']=True
            self.ui.backwardOrderSpinBox.setEnabled(False)
        else:
            self.updated_disabled_key_dict['BACKWARD']=False
            self.ui.backwardOrderSpinBox.setEnabled(True)

        if self.ui.leftCheckBox.isChecked():
            self.updated_disabled_key_dict['LEFT']=True
            self.ui.leftTurnOrderSpinBox.setEnabled(False)
        else:
            self.updated_disabled_key_dict['LEFT']=False
            self.ui.leftTurnOrderSpinBox.setEnabled(True)

        if self.ui.rightCheckBox.isChecked():
            self.updated_disabled_key_dict['RIGHT']=True
            self.ui.rightTurnOrderSpinBox.setEnabled(False)
        else:
            self.updated_disabled_key_dict['RIGHT']=False
            self.ui.rightTurnOrderSpinBox.setEnabled(True)

        if self.ui.leftTurnForwardCheckBox.isChecked():
            self.updated_disabled_key_dict['LEFT_TURN_FORWARD']=True
            self.ui.l_turn_fOrderSpinBox.setEnabled(False)
        else:
            self.updated_disabled_key_dict['LEFT_TURN_FORWARD']=False
            self.ui.l_turn_fOrderSpinBox.setEnabled(True)

        if self.ui.rightTurnForwardCheckBox.isChecked():
            self.updated_disabled_key_dict['RIGHT_TURN_FORWARD']=True
            self.ui.r_turn_fOrderSpinBox.setEnabled(False)
        else:
            self.updated_disabled_key_dict['RIGHT_TURN_FORWARD']=False
            self.ui.r_turn_fOrderSpinBox.setEnabled(True)

        if self.ui.leftTurnBackwardCheckBox.isChecked():
            self.updated_disabled_key_dict['LEFT_TURN_BACKWARD']=True
            self.ui.l_turn_bOrderSpinBox.setEnabled(False)
        else:
            self.updated_disabled_key_dict['LEFT_TURN_BACKWARD']=False
            self.ui.l_turn_bOrderSpinBox.setEnabled(True)

        if self.ui.rightTurnBackwardCheckBox.isChecked():
            self.updated_disabled_key_dict['RIGHT_TURN_BACKWARD']=True
            self.ui.r_turn_bOrderSpinBox.setEnabled(False)
        else:
            self.updated_disabled_key_dict['RIGHT_TURN_BACKWARD']=False
            self.ui.r_turn_bOrderSpinBox.setEnabled(True)

        if self.ui.accelerationCheckBox.isChecked():
            self.updated_disabled_key_dict['ACCELERATION']=True
            self.ui.acclOrderSpinBox.setEnabled(False)
        else:
            self.updated_disabled_key_dict['ACCELERATION']=False
            self.ui.acclOrderSpinBox.setEnabled(True)

        if self.ui.brakeCheckBox.isChecked():
            self.updated_disabled_key_dict['BRAKE']=True
            self.ui.brakeOrderSpinBox.setEnabled(False)
        else:
            self.updated_disabled_key_dict['BRAKE']=False
            self.ui.brakeOrderSpinBox.setEnabled(True)

        # NOTE: The actual dictionaries for disabling and key order are not updated here (Copies are used in functions)
        # Generate the new key list
        self.keys_comb_len, _ = self.generate_allowed_key_list(inp_disb_dict=self.updated_disabled_key_dict)

        # Redistribute the key order according to the new disabled key dictionary
        if redistribute is True:
            self.redistribute_key_order(inp_disb_dict=self.updated_disabled_key_dict,
                                        inp_order_dict=self.updated_key_order_dict)

        # Check for redundant order value
        retval=self.check_for_same_key_orders(inp_disb_dict=self.updated_disabled_key_dict,
                                              inp_order_dict=self.updated_key_order_dict)
        if retval is False:
            print("Same order keys found. Redistributing...")

            self.redistribute_key_order(inp_disb_dict=self.updated_disabled_key_dict,
                                        inp_order_dict=self.updated_key_order_dict)
            print("Redistributed")

        # Set the spin box maximum value
        self.ui.forwardOrderSpinBox.setRange(0, self.keys_comb_len)
        self.ui.backwardOrderSpinBox.setRange(0, self.keys_comb_len)
        self.ui.leftTurnOrderSpinBox.setRange(0, self.keys_comb_len)
        self.ui.rightTurnOrderSpinBox.setRange(0, self.keys_comb_len)
        self.ui.l_turn_fOrderSpinBox.setRange(0, self.keys_comb_len)
        self.ui.r_turn_fOrderSpinBox.setRange(0, self.keys_comb_len)
        self.ui.l_turn_bOrderSpinBox.setRange(0, self.keys_comb_len)
        self.ui.r_turn_bOrderSpinBox.setRange(0, self.keys_comb_len)
        self.ui.acclOrderSpinBox.setRange(0, self.keys_comb_len)
        self.ui.brakeOrderSpinBox.setRange(0, self.keys_comb_len)

        # Set the value for the spin box
        self.ui.forwardOrderSpinBox.setValue(self.updated_key_order_dict['FORWARD'])
        self.ui.backwardOrderSpinBox.setValue(self.updated_key_order_dict['BACKWARD'])
        self.ui.leftTurnOrderSpinBox.setValue(self.updated_key_order_dict['LEFT'])
        self.ui.rightTurnOrderSpinBox.setValue(self.updated_key_order_dict['RIGHT'])
        self.ui.l_turn_fOrderSpinBox.setValue(self.updated_key_order_dict['LEFT_TURN_FORWARD'])
        self.ui.r_turn_fOrderSpinBox.setValue(self.updated_key_order_dict['RIGHT_TURN_FORWARD'])
        self.ui.l_turn_bOrderSpinBox.setValue(self.updated_key_order_dict['LEFT_TURN_BACKWARD'])
        self.ui.r_turn_bOrderSpinBox.setValue(self.updated_key_order_dict['RIGHT_TURN_BACKWARD'])
        self.ui.acclOrderSpinBox.setValue(self.updated_key_order_dict['ACCELERATION'])
        self.ui.brakeOrderSpinBox.setValue(self.updated_key_order_dict['BRAKE'])

    def get_key_cfg_from_meta_file(self,file_name,file_path):  # Function to get disabled key and order from meta file

        # If the meta file for the current numpy train file exists
        if self.train_data_saver.open_meta_file_read_only(file_name,file_path) is True:
            # Read the file to get configuration information about the numpy file
            # The meta file is automatically closed her no need to manually close it
            self.parent_file_name,self.meta_key_order,self.meta_disabled_key=self.train_data_saver.read_cfg_from_meta_file()

            self.ko_comp_failed=False
            self.dk_comp_failed=False

            if self.train_data_saver.compare_dicts(self.move_button_press_order_dict,self.meta_key_order) is True:
                self.display_info(self.app_msg,"Key order configuration matches")
                self.ko_comp_failed=False
            else:
                self.display_info(self.app_msg,"Key order configuration does not match")
                self.ko_comp_failed=True

            if self.train_data_saver.compare_dicts(self.disabled_button_press_dict,self.meta_disabled_key) is True:
                self.display_info(self.app_msg,"Disabled key configuration matches")
                self.dk_comp_failed=False
            else:
                self.display_info(self.app_msg,"Disabled key configuration does not match")
                self.dk_comp_failed=True

            # Warn the user that the meta file does not match this is BAD
            if self.ko_comp_failed or self.dk_comp_failed:
                retval=self.show_key_cfg_mismatch_warning()  # Meta file key config is different from current one

                # If the user pressed ok apply the parsed meta file dictionary to the main dictionary
                # Update the GUI to display the new updated disabled key / order configuration
                if retval == QtGui.QMessageBox.Ok:
                    self.display_info(self.app_msg,"Applying the disabled key and order settings from meta file")

                    self.update_disabled_key_from_meta_file()  # The settings are applied from the meta file

                    return True  # User took desired action

                else:
                    self.display_info(self.app_msg,"Applying the disabled key and order settings was cancelled."
                                                   " Load the correct training data set numpy file.")

                    return False  # User did not take required action

            else:
                return True  # No errors occurred when checking the meta file

        else:
            # Show a warning dialog saying that meta file for the current numpy file does not exist
            # Ask the user whether to create a new meta file using the current key config for the current numpy file

            retval=self.show_create_key_cfg_file_warning() # Asking whether meta file should be created

            if retval == QtGui.QMessageBox.Ok:
                self.train_data_saver.open_meta_file_write_only(file_name,file_path)

                # The file is automatically closed inside the function
                self.train_data_saver.write_cfg_to_meta_file(paren_file_name=self.train_file_name,
                                                             key_order_dict=self.move_button_press_order_dict,
                                                             disable_key_dict=self.disabled_button_press_dict)

                self.display_info(self.app_msg,"The meta file was created")

                return True  # The meta file was created and was the desired action

            else:
                self.display_info(self.app_msg,"The meta file was not created. Please change the disabled key and order"
                                               " settings and apply the new settings to create the meta file")

                return False  # The meta file was not created and therefore is not a desired action

    def update_disabled_key_from_meta_file(self):  # Function to update the key configuration from meta file

        # Update the values from the meta file
        self.disabled_button_press_dict=self.meta_disabled_key
        self.move_button_press_order_dict=self.meta_key_order

        # Update the UI elements from the meta file
        self.ui.forwardCheckBox.setChecked(self.disabled_button_press_dict['FORWARD'])
        self.ui.backwardCheckBox.setChecked(self.disabled_button_press_dict['BACKWARD'])
        self.ui.leftCheckBox.setChecked(self.disabled_button_press_dict['LEFT'])
        self.ui.rightCheckBox.setChecked(self.disabled_button_press_dict['RIGHT'])
        self.ui.leftTurnForwardCheckBox.setChecked(self.disabled_button_press_dict['LEFT_TURN_FORWARD'])
        self.ui.rightTurnForwardCheckBox.setChecked(self.disabled_button_press_dict['RIGHT_TURN_FORWARD'])
        self.ui.leftTurnBackwardCheckBox.setChecked(self.disabled_button_press_dict['LEFT_TURN_BACKWARD'])
        self.ui.rightTurnBackwardCheckBox.setChecked(self.disabled_button_press_dict['RIGHT_TURN_BACKWARD'])
        self.ui.accelerationCheckBox.setChecked(self.disabled_button_press_dict['ACCELERATION'])
        self.ui.brakeCheckBox.setChecked(self.disabled_button_press_dict['BRAKE'])

        self.ui.forwardOrderSpinBox.setValue(self.move_button_press_order_dict['FORWARD'])
        self.ui.backwardOrderSpinBox.setValue(self.move_button_press_order_dict['BACKWARD'])
        self.ui.leftTurnOrderSpinBox.setValue(self.move_button_press_order_dict['LEFT'])
        self.ui.rightTurnOrderSpinBox.setValue(self.move_button_press_order_dict['RIGHT'])
        self.ui.l_turn_fOrderSpinBox.setValue(self.move_button_press_order_dict['LEFT_TURN_FORWARD'])
        self.ui.r_turn_fOrderSpinBox.setValue(self.move_button_press_order_dict['RIGHT_TURN_FORWARD'])
        self.ui.l_turn_bOrderSpinBox.setValue(self.move_button_press_order_dict['LEFT_TURN_BACKWARD'])
        self.ui.r_turn_bOrderSpinBox.setValue(self.move_button_press_order_dict['RIGHT_TURN_BACKWARD'])
        self.ui.acclOrderSpinBox.setValue(self.move_button_press_order_dict['ACCELERATION'])
        self.ui.brakeOrderSpinBox.setValue(self.move_button_press_order_dict['BRAKE'])

    def show_key_cfg_mismatch_warning(self):  # Called when the key config does not match
        msg = QtGui.QMessageBox()

        msg.setIcon(QtGui.QMessageBox.Warning)
        msg.setWindowIcon(QtGui.QIcon("bot_icon.jpg"))
        msg.setWindowTitle("Key configuration mismatch warning")
        msg.setText("Key configuration mismatch detected. Do you want to use the key configuration "
                    "associated with the loaded numpy file?")

        msg.setInformativeText("The current disabled key and order settings does not match with the "
                               "training data set numpy file : "+str(self.train_file_name)+". "
                               "If the file name of the numpy (.npy) file was changed before, please change the "
                               "associated  meta file to the same name otherwise incorrect key "
                               "configuration may be loaded.")

        msg.setStandardButtons(QtGui.QMessageBox.Cancel | QtGui.QMessageBox.Ok)
        msg.setDefaultButton(QtGui.QMessageBox.Cancel)

        return msg.exec_()  # The return value from executing the above message

    def show_create_key_cfg_file_warning(self):
        msg = QtGui.QMessageBox()

        msg.setIcon(QtGui.QMessageBox.Warning)
        msg.setWindowIcon(QtGui.QIcon("bot_icon.jpg"))
        msg.setWindowTitle("Create new meta file")
        msg.setText("Create a new meta file using the current disabled key and order settings?")

        msg.setInformativeText("A new meta file using the current disabled key and order settings will be created "
                               "for the current training data numpy file : "+str(self.train_file_name)+". "
                               "Please ensure that the numpy file is new and does not have any previous inputs.")

        msg.setStandardButtons(QtGui.QMessageBox.Cancel | QtGui.QMessageBox.Ok)
        msg.setDefaultButton(QtGui.QMessageBox.Cancel)

        return msg.exec_()  # The return value from executing the above message

    def update_copies_of_key_order_disable_dict(self):  # Update the dictionary copies to the values set by the user
        self.updated_key_order_dict=self.move_button_press_order_dict.copy()  # Shallow copy is used
        self.updated_disabled_key_dict=self.disabled_button_press_dict.copy()  # Shallow copy is used

    @staticmethod
    def show_key_order_warning():
        msg = QtGui.QMessageBox()

        msg.setIcon(QtGui.QMessageBox.Warning)
        msg.setWindowIcon(QtGui.QIcon("bot_icon.jpg"))
        msg.setWindowTitle("Enabled keys sharing same order")
        msg.setText("Please use unique order values for the enabled keys. Key order will be redistributed.")

        msg.setStandardButtons(QtGui.QMessageBox.Ok)
        msg.setDefaultButton(QtGui.QMessageBox.Ok)

        return msg.exec_()  # The return value from executing the above message

    @staticmethod
    def generate_allowed_key_list(inp_disb_dict):  # Function to create a new allowed key list
        inp_mov_list=[]  # Reset the list that contains the location for the keys

        for _,value in inp_disb_dict.items():
            if value is False:  # If the key/key combination is not disabled
                inp_mov_list.append(0)  # Add a location for the key

        return (len(inp_mov_list)-1),inp_mov_list

    @staticmethod
    def redistribute_key_order(inp_disb_dict,inp_order_dict):  # Function to redistribute the key order
        order = 0
        for key, value in inp_disb_dict.items():
            if inp_disb_dict[key] is False:  # If the key is not disabled
                inp_order_dict[key] = order  # Then assign a new order value
                order += 1  # Increment the value
            else:
                inp_order_dict[key] = 0  # Set to 0 if disabled

    @staticmethod
    def check_for_same_key_orders(inp_disb_dict,inp_order_dict):  # Function to check for shared key value
        check_list = []
        for key, value in inp_order_dict.items():
            if inp_disb_dict[key] is False:  # Do this for only keys not disabled
                if value not in check_list:  # If the value is not in the list append it
                    check_list.append(value)
                else:
                    return False  # Keys share the same order value

        return True  # No Keys share the same order value

    # Pan and tilt settings toolbox functions
    def pan_tilt_config_set_clicked(self):
        self.pan_tilt_limit_dict['PAN_LEFT'] = self.ui.panLeftSpinBox.value()
        self.pan_tilt_limit_dict['PAN_RIGHT'] = self.ui.panRightSpinBox.value()
        self.pan_tilt_limit_dict['TILT_DOWN'] = self.ui.tiltDownSpinBox.value()
        self.pan_tilt_limit_dict['TILT_UP'] = self.ui.tiltUpSpinBox.value()

        self.pan_scaler = self.ui.panSensitivitySpinBox.value()
        self.tilt_scaler = self.ui.tiltSensivitySpinBox.value()

        # Calculate the new degree per pixel
        self.calc_pan_tilt_degree_per_pixel()

        self.enable_mouse_pan_tilt = self.ui.panTiltEnableCheckBox.isChecked()

        enb_disp_str=''
        if self.enable_mouse_pan_tilt is True:
            enb_disp_str="Enabled"
        else:
            enb_disp_str="Disabled"

        self.config.set('Pan_tilt_cfg','pan_left',str(self.pan_tilt_limit_dict['PAN_LEFT']))
        self.config.set('Pan_tilt_cfg','pan_right',str(self.pan_tilt_limit_dict['PAN_RIGHT']))
        self.config.set('Pan_tilt_cfg','tilt_down',str(self.pan_tilt_limit_dict['TILT_DOWN']))
        self.config.set('Pan_tilt_cfg','tilt_up',str(self.pan_tilt_limit_dict['TILT_UP']))

        self.config.set('Pan_tilt_cfg','pan_scaler',str(self.pan_scaler))
        self.config.set('Pan_tilt_cfg','tilt_scaler',str(self.tilt_scaler))

        self.config.set('Pan_tilt_cfg','pan_tilt_enable',str(int(self.enable_mouse_pan_tilt)))

        self.display_info(self.app_msg,"New pan tilt settings applied")
        self.display_info(self.app_msg,"Pan left maximum: "+str(self.pan_tilt_limit_dict['PAN_LEFT']))
        self.display_info(self.app_msg,"Pan right maximum: "+str(self.pan_tilt_limit_dict['PAN_RIGHT']))
        self.display_info(self.app_msg,"Tilt down maximum: "+str(self.pan_tilt_limit_dict['TILT_DOWN']))
        self.display_info(self.app_msg,"Tilt up: maximum"+str(self.pan_tilt_limit_dict['TILT_UP']))

        self.display_info(self.app_msg,"Pan sensitivity: "+str(self.pan_scaler))
        self.display_info(self.app_msg,"Tilt sensitivity: "+str(self.tilt_scaler))

        self.display_info(self.app_msg,"Camera pan and tilt: "+enb_disp_str)

        self.write_to_config_file()

    def pan_tilt_config_reset_clicked(self):
        self.get_pan_tilt_settings_cfg()

        self.display_info(self.app_msg,"Pan tilt settings have been reset to previous values")

    def get_pan_tilt_settings_cfg(self):
        self.pan_tilt_limit_dict['PAN_LEFT']=self.config.getint('Pan_tilt_cfg','pan_left')
        self.pan_tilt_limit_dict['PAN_RIGHT']=self.config.getint('Pan_tilt_cfg','pan_right')
        self.pan_tilt_limit_dict['TILT_DOWN']=self.config.getint('Pan_tilt_cfg','tilt_down')
        self.pan_tilt_limit_dict['TILT_UP']=self.config.getint('Pan_tilt_cfg','tilt_up')

        self.pan_scaler=self.config.getfloat('Pan_tilt_cfg','pan_scaler')
        self.tilt_scaler=self.config.getfloat('Pan_tilt_cfg','tilt_scaler')

        # Calculate the new degree per pixel
        self.calc_pan_tilt_degree_per_pixel()

        self.enable_mouse_pan_tilt=self.config.getboolean('Pan_tilt_cfg','pan_tilt_enable')

        self.ui.panLeftSpinBox.setValue(self.pan_tilt_limit_dict['PAN_LEFT'])
        self.ui.panRightSpinBox.setValue(self.pan_tilt_limit_dict['PAN_RIGHT'])
        self.ui.tiltDownSpinBox.setValue(self.pan_tilt_limit_dict['TILT_DOWN'])
        self.ui.tiltUpSpinBox.setValue(self.pan_tilt_limit_dict['TILT_UP'])

        self.ui.panSensivitySlider.setValue(self.pan_scaler)
        self.ui.tiltSensivitySlider.setValue(self.tilt_scaler)
        self.ui.panSensitivitySpinBox.setValue(self.pan_scaler)
        self.ui.tiltSensivitySpinBox.setValue(self.tilt_scaler)

        self.ui.panTiltEnableCheckBox.setChecked(self.enable_mouse_pan_tilt)

        self.enable_pan_tilt_checkbox_changed()

    def calc_pan_tilt_degree_per_pixel(self):  # Function to calculate the degree per pixel for pan and tilt
        self.dpp_tilt = (180.0 / float(
            self.pan_tilt_limit_dict['TILT_UP'] - self.pan_tilt_limit_dict['TILT_DOWN'])) / self.tilt_scaler
        self.dpp_pan = (180.0 / float(
            self.pan_tilt_limit_dict['PAN_RIGHT'] - self.pan_tilt_limit_dict['PAN_LEFT'])) / self.pan_scaler

        return self.dpp_pan,self.dpp_tilt

    def pan_sensitivity_slider_changed(self):
        self.ui.panSensitivitySpinBox.blockSignals(True)
        self.ui.panSensitivitySpinBox.setValue(self.ui.panSensivitySlider.value())
        self.ui.panSensitivitySpinBox.blockSignals(False)

    def pan_sensitivity_spinbox_changed(self):
        self.ui.panSensivitySlider.blockSignals(True)
        self.ui.panSensivitySlider.setValue(self.ui.panSensitivitySpinBox.value())
        self.ui.panSensivitySlider.blockSignals(False)

    def tilt_sensitivity_slider_changed(self):
        self.ui.tiltSensivitySpinBox.blockSignals(True)
        self.ui.tiltSensivitySpinBox.setValue(self.ui.tiltSensivitySlider.value())
        self.ui.tiltSensivitySpinBox.blockSignals(False)

    def tilt_sensitivity_spinbox_changed(self):
        self.ui.tiltSensivitySlider.blockSignals(True)
        self.ui.tiltSensivitySlider.setValue(self.ui.tiltSensivitySpinBox.value())
        self.ui.tiltSensivitySlider.blockSignals(False)

    def enable_pan_tilt_checkbox_changed(self):

        enb_disb=self.ui.panTiltEnableCheckBox.isChecked()

        enb_disb=not enb_disb

        self.ui.panLeftSpinBox.setDisabled(enb_disb)
        self.ui.panRightSpinBox.setDisabled(enb_disb)
        self.ui.tiltDownSpinBox.setDisabled(enb_disb)
        self.ui.tiltUpSpinBox.setDisabled(enb_disb)

        self.ui.panSensivitySlider.setDisabled(enb_disb)
        self.ui.tiltSensivitySlider.setDisabled(enb_disb)

        self.ui.panSensitivitySpinBox.setDisabled(enb_disb)
        self.ui.tiltSensivitySpinBox.setDisabled(enb_disb)

    # Setting all the threshold at once
    def set_all_thresholds(self):
        self.display_info(self.app_msg,"All the thresholds will be applied")

        # Call the functions for the set buttons
        self.gas_threshold_set_clicked()
        self.particle_threshold_set_clicked()

    # Gas sensor threshold settings
    def gas_threshold_set_clicked(self):
        self.eCO2_thresh=self.ui.eCO2ThresholdSpinBox.value()
        self.TVOC_thresh=self.ui.tvocThresholdSpinBox.value()

        self.display_info(self.app_msg,"New gas sensor thresholds will be applied")
        self.display_info(self.app_msg,"Equivalent CO2 threshold (eCO2): "+str(self.eCO2_thresh)+" ppm")
        self.display_info(self.app_msg,"Total Volatile Organic Compound threshold (TVOC): "+str(self.TVOC_thresh)+" ppb")

        self.config.set('Gas_thresh','eco2',str(self.eCO2_thresh))
        self.config.set('Gas_thresh','tvoc',str(self.TVOC_thresh))

        self.write_to_config_file()

        cmd=self.format_command(self.socket_send_header_dict['GAS_THR'],[self.eCO2_thresh,self.TVOC_thresh])

        # Command to set the thresholds should be sent
        self.send_socket_commands(cmd)

    def gas_threshold_reset_clicked(self):
        self.get_gas_threshold_setting_cfg()

        self.display_info(self.app_msg,"The gas sensor threshold have been set to the previous values")

    def get_gas_threshold_setting_cfg(self):
        self.eCO2_thresh=self.config.getint('Gas_thresh','eco2')
        self.TVOC_thresh=self.config.getint('Gas_thresh','tvoc')

        self.ui.eCO2ThresholdSpinBox.setValue(self.eCO2_thresh)
        self.ui.tvocThresholdSpinBox.setValue(self.TVOC_thresh)

    # Particle sensor threshold settings
    def particle_threshold_set_clicked(self):
        self.Red_thresh=self.ui.redThresholdSpinBox.value()
        self.Green_thresh=self.ui.greenThresholdSpinBox.value()
        self.IR_thresh=self.ui.irThresholdSpinBox.value()

        self.display_info(self.app_msg,"New particles sensor thresholds will be applied")
        self.display_info(self.app_msg,"Red threshold: "+str(self.Red_thresh))
        self.display_info(self.app_msg,"Green threshold: " + str(self.Green_thresh))
        self.display_info(self.app_msg,"IR threshold: " + str(self.IR_thresh))

        self.config.set('Particle_thresh','red',str(self.Red_thresh))
        self.config.set('Particle_thresh','green',str(self.Green_thresh))
        self.config.set('Particle_thresh','ir',str(self.IR_thresh))

        self.write_to_config_file()

        cmd=self.format_command(self.socket_send_header_dict['PAR_THR'],[self.Red_thresh,self.Green_thresh
                                                                         ,self.IR_thresh])
        # Command should be sent to set the particle thresholds
        self.send_socket_commands(cmd)

    def particle_threshold_reset_clicked(self):
        self.get_particle_threshold_settings_cfg()

        self.display_info(self.app_msg,"Particle sensor have been set to the previous values")

    def get_particle_threshold_settings_cfg(self):
        self.Red_thresh=self.config.getint('Particle_thresh','red')
        self.Green_thresh = self.config.getint('Particle_thresh', 'green')
        self.IR_thresh = self.config.getint('Particle_thresh', 'ir')

        self.ui.redThresholdSpinBox.setValue(self.Red_thresh)
        self.ui.greenThresholdSpinBox.setValue(self.Green_thresh)
        self.ui.irThresholdSpinBox.setValue(self.IR_thresh)

    # SMS alert settings
    def sms_alert_set_clicked(self):
        self.sms_number=self.ui.smsNumberLineEdit.text()

        enb_string=""

        if self.ui.smsAlertEnableCheckBox.isChecked() is True:
            self.sms_enabled=1
            enb_string="Enabled"
        else:
            self.sms_enabled=0
            enb_string="Disabled"

        self.display_info(self.app_msg,"New SMS alert settings will be applied")
        self.display_info(self.app_msg,"SMS alert number: "+str(self.sms_number))
        self.display_info(self.app_msg,"SMS enabled: "+enb_string)

        self.config.set('SMS_alert','alert_number',str(self.sms_number))
        self.config.set('SMS_alert','alert_enable',str(self.sms_enabled))

        self.write_to_config_file()

        cmd=self.format_command(self.socket_send_header_dict['SMS'],[self.sms_number,self.sms_enabled])

        # Command should be sent to set the SMS settings
        self.send_socket_commands(cmd)

    def sms_alert_reset_clicked(self):
        self.get_sms_setting_cfg()

        self.display_info(self.app_msg,"SMS alert settings have been set to the previous values")

    def get_sms_setting_cfg(self):
        self.sms_number=self.config.get('SMS_alert','alert_number')
        self.sms_enabled=self.config.getint('SMS_alert','alert_enable')

        self.ui.smsNumberLineEdit.setText(self.sms_number)

        if self.sms_enabled == 1:
            self.ui.smsAlertEnableCheckBox.setChecked(True)
        else:
            self.ui.smsAlertEnableCheckBox.setChecked(False)

        self.enable_sms_alert_checkbox_changed()

    def enable_sms_alert_checkbox_changed(self):

        enb_disb=self.ui.smsAlertEnableCheckBox.isChecked()

        self.ui.smsNumberLineEdit.setDisabled(not enb_disb)

    # Keyboard presses detection
    def keyPressEvent(self, event):  # Event gets called when a key is pressed
        if event.isAutoRepeat():
            return

        pressed_key=event.key()
        self.get_key_press(pressed_key)
        self.get_and_handle_function_key_presses(pressed_key)

        self.start_button_press_timer()  # Start the timer

        event.accept()

    def keyReleaseEvent(self, event):  # Event gets called when a key is released
        if event.isAutoRepeat():
            return

        released_key=event.key()
        self.get_key_release(released_key)

        self.stop_button_press_timer()  # Stop the timer if and only if all the buttons are released

        event.accept()

    def get_key_press(self,key_code):  # Function to get which key is pressed

            if key_code == QtCore.Qt.Key_W:  # For forward motion
                self.key_pressed_dict['W']=True
                self.ui.forwardButton.setDown(True)

            elif key_code == QtCore.Qt.Key_A:  # For left turn and forward/backward + left turn
                self.key_pressed_dict['A']=True
                self.ui.leftButton.setDown(True)

            elif key_code == QtCore.Qt.Key_S:  # For backward motion
                self.key_pressed_dict['S']=True
                self.ui.backwardButton.setDown(True)

            elif key_code == QtCore.Qt.Key_D:  # For right turn and forward/backward + right turn
                self.key_pressed_dict['D']=True
                self.ui.rightButton.setDown(True)

            elif key_code == QtCore.Qt.Key_Shift: # For increasing the speed
                self.key_pressed_dict['SHIFT']=True
                self.ui.increaseSpeedButton.setDown(True)

            elif key_code == QtCore.Qt.Key_Control: # For braking
                self.key_pressed_dict['CTRL']=True
                self.ui.brakeButton.setDown(True)

    def get_key_release(self,key_code):  # Function to get which key was released

            if key_code == QtCore.Qt.Key_W:  # For forward motion
                self.key_pressed_dict['W']=False
                self.ui.forwardButton.setDown(False)

            elif key_code == QtCore.Qt.Key_A:  # For left turn and forward/backward + left turn
                self.key_pressed_dict['A']=False
                self.ui.leftButton.setDown(False)

            elif key_code == QtCore.Qt.Key_S:  # For backward motion
                self.key_pressed_dict['S']=False
                self.ui.backwardButton.setDown(False)

            elif key_code == QtCore.Qt.Key_D:  # For right turn and forward/backward + right turn
                self.key_pressed_dict['D']=False
                self.ui.rightButton.setDown(False)

            elif key_code == QtCore.Qt.Key_Shift:  # For increasing the speed
                self.key_pressed_dict['SHIFT']=False
                self.ui.increaseSpeedButton.setDown(False)

            elif key_code == QtCore.Qt.Key_Control:  # For braking
                self.key_pressed_dict['CTRL']=False
                self.ui.brakeButton.setDown(False)

    def start_button_press_timer(self): # Function that to start the button press timer
        if not self.button_handle_timer.isActive():  # If the timer is not active
            self.handle_key_presses()  # This sends the  first command instantaneously
            self.button_handle_timer.start()  # Start the timer

    def stop_button_press_timer(self):  # Function stop to button press timer
        for _, value in self.key_pressed_dict.items():  # Iterating over the dictionary
            if value is True:  # If any of the value is true do not stop the timer
                return

        if self.button_handle_timer.isActive():  # If the timer is active
            self.button_handle_timer.stop()  # Then stop the timer as none of the buttons are active

    def handle_key_presses(self):  # This function gets called by the button press timer at the set interval

        cmd_str=""

        if self.key_pressed_dict['CTRL']:  # Braking
            self.brake=1

            # A command is sent to enable the brakes which keeps the robot from moving
            cmd_str=self.format_command(self.socket_send_header_dict['MOVE'],[self.movement_cmd_dict['BWRD'],
                                                                              self.accl,self.brake])

            # Command is send if not disabled and not send if it is disabled and the send command is also disabled
            if not self.disabled_button_press_dict['BRAKE'] or not self.disable_send_command:
                self.send_socket_commands(cmd_str)
        else:
            self.brake=0

        if self.key_pressed_dict['SHIFT']:  # Increasing speed
            if not self.disabled_button_press_dict['ACCELERATION'] or not self.disable_send_command:
                self.accl=1
        else:
            self.accl=0

        if not self.key_pressed_dict['A'] and not self.key_pressed_dict['D']:  # Forward/Backward
            if self.key_pressed_dict['W']:
                cmd_str=self.format_command(self.socket_send_header_dict['MOVE'],[self.movement_cmd_dict['FRWD']
                                                                                  ,self.accl,self.brake])  # Formatting the command

                if not self.disabled_button_press_dict['FORWARD'] or not self.disable_send_command:
                    self.send_socket_commands(cmd_str)  # Sending the command

            elif self.key_pressed_dict['S']:
                cmd_str=self.format_command(self.socket_send_header_dict['MOVE'],[self.movement_cmd_dict['BWRD']
                                                                                  ,self.accl,self.brake])

                if not self.disabled_button_press_dict['BACKWARD'] or not self.disable_send_command:
                    self.send_socket_commands(cmd_str)  # Sending the command

        elif not self.key_pressed_dict['W'] and not self.key_pressed_dict['S']:  # Left/Right
            if self.key_pressed_dict['A']:
                cmd_str = self.format_command(self.socket_send_header_dict['MOVE'], [self.movement_cmd_dict['LFT']
                                                                                     ,self.accl,self.brake])

                if not self.disabled_button_press_dict['LEFT'] or not self.disable_send_command:
                    self.send_socket_commands(cmd_str)  # Sending the command

            elif self.key_pressed_dict['D']:
                cmd_str = self.format_command(self.socket_send_header_dict['MOVE'], [self.movement_cmd_dict['RGT']
                                                                                     ,self.accl,self.brake])

                if not self.disabled_button_press_dict['RIGHT'] or not self.disable_send_command:
                    self.send_socket_commands(cmd_str)  # Sending the command

        # Turning with Forward/Backward
        elif self.key_pressed_dict['W'] and self.key_pressed_dict['A']:
            cmd_str = self.format_command(self.socket_send_header_dict['MOVE'], [self.movement_cmd_dict['L_TURN_F']
                                                                                 ,self.accl,self.brake])

            if not self.disabled_button_press_dict['LEFT_TURN_FORWARD'] or not self.disable_send_command:
                self.send_socket_commands(cmd_str)  # Sending the command

        elif self.key_pressed_dict['W'] and self.key_pressed_dict['D']:
            cmd_str = self.format_command(self.socket_send_header_dict['MOVE'], [self.movement_cmd_dict['R_TURN_F']
                                                                                 ,self.accl,self.brake])

            if not self.disabled_button_press_dict['RIGHT_TURN_FORWARD'] or not self.disable_send_command:
                self.send_socket_commands(cmd_str)  # Sending the command

        elif self.key_pressed_dict['S'] and self.key_pressed_dict['A']:
            cmd_str = self.format_command(self.socket_send_header_dict['MOVE'], [self.movement_cmd_dict['L_TURN_B']
                                                                                 ,self.accl,self.brake])

            if not self.disabled_button_press_dict['LEFT_TURN_BACKWARD'] or not self.disable_send_command:
                self.send_socket_commands(cmd_str)  # Sending the command

        elif self.key_pressed_dict['S'] and self.key_pressed_dict['D']:
            cmd_str = self.format_command(self.socket_send_header_dict['MOVE'], [self.movement_cmd_dict['R_TURN_B']
                                                                                 ,self.accl,self.brake])

            if not self.disabled_button_press_dict['RIGHT_TURN_BACKWARD'] or not self.disable_send_command:
                self.send_socket_commands(cmd_str)  # Sending the command

    def is_keys_active(self):  # Function to check if any of the keys are pressed
        for _, value in self.key_pressed_dict.items():
            if value is True:
                return True

        return False

    def is_disabled_key_pressed(self):  # Function to check whether any disabled key/key combinations are pressed
        if not self.disabled_move_key_pressed:  # If any of the disabled key is pressed return false
            return False
        else:
            print("Disabled movement key or key combination pressed")
            return True

    def output_key_presses(self):  # Function to get the key presses in a one/multi hot array format

        """
        This function is used for the collection of control input from the user along with the
        frames and should be used in the function where the screen is updated. This reduces the
        problem of reading the variables at the wrong time and not getting the list updated with
        the latest key presses.
        """

        # Reset the disabled key pressed variable
        self.disabled_move_key_pressed=False

        # Reset the pressed button list
        self.move_button_press_list=self.default_move_list.copy()  # A shallow copy of the default list is used to reset

        if self.key_pressed_dict['CTRL']:  # Braking
            if not self.disabled_button_press_dict['BRAKE']:
                self.move_button_press_list[self.move_button_press_order_dict['BRAKE']] = 1
            else:
                self.disabled_move_key_pressed=True

        if self.key_pressed_dict['SHIFT']:  # Increasing speed
            if not self.disabled_button_press_dict['ACCELERATION']:
                self.move_button_press_list[self.move_button_press_order_dict['ACCELERATION']] = 1
            else:
                self.disabled_move_key_pressed=True

        if not self.key_pressed_dict['A'] and not self.key_pressed_dict['D']:  # Forward/Backward
            if self.key_pressed_dict['W']:
                if not self.disabled_button_press_dict['FORWARD']:
                    self.move_button_press_list[self.move_button_press_order_dict['FORWARD']] = 1
                else:
                    self.disabled_move_key_pressed = True

            elif self.key_pressed_dict['S']:
                if not self.disabled_button_press_dict['BACKWARD']:
                    self.move_button_press_list[self.move_button_press_order_dict['BACKWARD']] = 1
                else:
                    self.disabled_move_key_pressed = True

        elif not self.key_pressed_dict['W'] and not self.key_pressed_dict['S']:  # Left/Right
            if self.key_pressed_dict['A']:
                if not self.disabled_button_press_dict['LEFT']:
                    self.move_button_press_list[self.move_button_press_order_dict['LEFT']] = 1
                else:
                    self.disabled_move_key_pressed = True

            elif self.key_pressed_dict['D']:
                if not self.disabled_button_press_dict['RIGHT']:
                    self.move_button_press_list[self.move_button_press_order_dict['RIGHT']] = 1
                else:
                    self.disabled_move_key_pressed = True

        # Turning with Forward/Backward
        elif self.key_pressed_dict['W'] and self.key_pressed_dict['A']:
            if not self.disabled_button_press_dict['LEFT_TURN_FORWARD']:
                self.move_button_press_list[self.move_button_press_order_dict['LEFT_TURN_FORWARD']] = 1
            else:
                self.disabled_move_key_pressed=True

        elif self.key_pressed_dict['W'] and self.key_pressed_dict['D']:
            if not self.disabled_button_press_dict['RIGHT_TURN_FORWARD']:
                self.move_button_press_list[self.move_button_press_order_dict['RIGHT_TURN_FORWARD']] = 1
            else:
                self.disabled_move_key_pressed=True

        elif self.key_pressed_dict['S'] and self.key_pressed_dict['A']:
            if not self.disabled_button_press_dict['LEFT_TURN_BACKWARD']:
                self.move_button_press_list[self.move_button_press_order_dict['LEFT_TURN_BACKWARD']] = 1
            else:
                self.disabled_move_key_pressed=True

        elif self.key_pressed_dict['S'] and self.key_pressed_dict['D']:
            if not self.disabled_button_press_dict['RIGHT_TURN_BACKWARD']:
                self.move_button_press_list[self.move_button_press_order_dict['RIGHT_TURN_BACKWARD']] = 1
            else:
                self.disabled_move_key_pressed=True

        return self.move_button_press_list  # Return the list

    def handle_dnn_key_press(self,key_press_list):  # The function handle the key press prediction from DNN

        # Variable to hold the acceleration and brake
        brake=0
        accl=0

        self.key_pressed_dict=self.default_key_pressed_dict.copy()

        if not self.dnn_disabled_button_press_dict['BRAKE']:
            if key_press_list[self.dnn_move_button_press_order_dict['BRAKE']] == 1:
                brake=1
                self.key_pressed_dict['CTRL']=True
                self.handle_key_presses()
                return

        if not self.dnn_disabled_button_press_dict['ACCELERATION']:
            if key_press_list[self.dnn_move_button_press_order_dict['ACCELERATION']] == 1:
                accl=1
                self.key_pressed_dict['SHIFT']=True
                # Here return is not used as this used in combination with other key
                # This is not yet implemented

        if not self.dnn_disabled_button_press_dict['FORWARD']:
            if key_press_list[self.dnn_move_button_press_order_dict['FORWARD']] == 1:
                self.key_pressed_dict['W']=True
                self.handle_key_presses()
                return

        if not self.dnn_disabled_button_press_dict['BACKWARD']:
            if key_press_list[self.dnn_move_button_press_order_dict['BACKWARD']] == 1:
                self.key_pressed_dict['S']=True
                self.handle_key_presses()
                return

        if not self.dnn_disabled_button_press_dict['LEFT']:
            if key_press_list[self.dnn_move_button_press_order_dict['LEFT']] == 1:
                self.key_pressed_dict['A']=True
                self.handle_key_presses()
                return

        if not self.dnn_disabled_button_press_dict['RIGHT']:
            if key_press_list[self.dnn_move_button_press_order_dict['RIGHT']] == 1:
                self.key_pressed_dict['D']=True
                self.handle_key_presses()
                return

        if not self.dnn_disabled_button_press_dict['LEFT_TURN_FORWARD']:
            if key_press_list[self.dnn_move_button_press_order_dict['LEFT_TURN_FORWARD']] == 1:
                self.key_pressed_dict['W']=True
                self.key_pressed_dict['A']=True
                self.handle_key_presses()
                return

        if not self.dnn_disabled_button_press_dict['RIGHT_TURN_FORWARD']:
            if key_press_list[self.dnn_move_button_press_order_dict['RIGHT_TURN_FORWARD']] == 1:
                self.key_pressed_dict['W']=True
                self.key_pressed_dict['D']=True
                self.handle_key_presses()
                return

        if not self.dnn_disabled_button_press_dict['LEFT_TURN_BACKWARD']:
            if key_press_list[self.dnn_move_button_press_order_dict['LEFT_TURN_BACKWARD']] == 1:
                self.key_pressed_dict['S']=True
                self.key_pressed_dict['A']=True
                self.handle_key_presses()
                return

        if not self.dnn_disabled_button_press_dict['RIGHT_TURN_BACKWARD']:
            if key_press_list[self.dnn_move_button_press_order_dict['RIGHT_TURN_BACKWARD']] == 1:
                self.key_pressed_dict['S']=True
                self.key_pressed_dict['D']=True
                self.handle_key_presses()
                return

        if brake == 0 and accl == 0:  # If reached here no matches were found
            print("No matching keypress")
            return

    def get_and_handle_function_key_presses(self, key_code):  # Function to handle function key presses

        if key_code == QtCore.Qt.Key_F:  # This is for changing the frame type from normal to say thermal
            self.change_display_mode_by_key()

        elif key_code == QtCore.Qt.Key_R:  # This is to start recording the frames
            self.recording_start_stop()

        elif key_code == QtCore.Qt.Key_T:  # This is to start recording the training data
            self.training_data_recording_start_stop()

        elif key_code == QtCore.Qt.Key_N:  # For activating/deactivating the Neural Network
            self.neural_net_activate_deactivate()

    # Mouse movement over the video frame detection
    def eventFilter(self, source, event):  # This is for the mouse move event over the video frame
        if event.type() == QtCore.QEvent.MouseMove and source is self.ui.video_frame:
            if self.mouse_pressed_dict['LEFT_MB']:
                mouse_position=event.pos()
                self.mouse_pos_from_start(mouse_position.x(),mouse_position.y())  # Get the position from start
                self.calc_pan_tilt_angle(self.mouse_x,self.mouse_y)
                self.send_pan_tilt_command(self.pan_angle,self.tilt_angle)

        elif event.type() == QtCore.QEvent.MouseButtonPress:  # When the mouse button is pressed
            if event.button() == QtCore.Qt.LeftButton:
                self.mouse_pressed_dict['LEFT_MB']=True
                mouse_position=event.pos()
                # Store the starting position
                self.start_x=mouse_position.x()
                self.start_y=mouse_position.y()
                print("Left button pressed")

        elif event.type() == QtCore.QEvent.MouseButtonRelease:  # When the mouse button is released
            if event.button() == QtCore.Qt.LeftButton:
                self.mouse_pressed_dict['LEFT_MB']=False
                # Reset the value
                self.start_x=0
                self.start_y=0
                # Reset the pan and tilt value to the default
                self.pan_angle=self.pan_default
                self.tilt_angle=self.tilt_default
                self.send_pan_tilt_command(self.pan_angle,self.tilt_angle)
                print("Left button released")

        return QtGui.QMainWindow.eventFilter(self,source,event)  # This return is necessary

    def mouse_pos_from_start(self,mpx,mpy):  # Function to calculate the mouse postion from where the click happened
        self.mouse_x=mpx-self.start_x
        self.mouse_y=-(mpy-self.start_y)  # Negative of the y value to flip sign when going up

        return self.mouse_x,self.mouse_y

    def calc_pan_tilt_angle(self,mpx,mpy):  # Function to calculate the pan and tilt angle

        self.pan_disp=mpx*self.dpp_pan  # Get the angle
        self.pan_disp=round(self.pan_disp,0)  # Get the rounded value
        self.pan_angle=int(90+self.pan_disp)  # This is the actual angle to be sent

        if self.pan_angle <= self.pan_tilt_limit_dict['PAN_LEFT']:  # If the left limit is exceeded
            self.pan_angle=0  # Set to the minimum

        elif self.pan_angle >= self.pan_tilt_limit_dict['PAN_RIGHT']:  # If the right limit is exceeded
            self.pan_angle=180  # Set to the maximum

        self.tilt_disp=mpy*self.dpp_tilt
        self.tilt_disp=round(self.tilt_disp,0)
        self.tilt_angle=int(90+self.tilt_disp)

        if self.tilt_angle <= self.pan_tilt_limit_dict['TILT_DOWN']:  # If the down limit is reached
            self.tilt_angle=0  # Set to the minimum

        elif self.tilt_angle >= self.pan_tilt_limit_dict['TILT_UP']:  # If the up limit is reached
            self.tilt_angle=180  # Set to the maximum

        print("P= "+str(self.pan_angle)+" T= "+str(self.tilt_angle))

        # Command to pan and tilt the camera assembly should be sent here
        self.send_pan_tilt_command(self.pan_angle,self.tilt_angle)

    def send_pan_tilt_command(self,pan,tilt):  # Function to send the pan tilt command
        cmd=self.format_command(self.socket_send_header_dict['CAM_PT'],[pan,tilt])

        if self.enable_mouse_pan_tilt is True:  # Send commands only if enabled
            self.send_socket_commands(cmd)  # Send the pan and tilt command

    # Formatting of command
    @staticmethod
    def format_command(cmd_type,data_list):  # Function to format command and command data
        start_marker="$"
        end_marker="#"

        cmd_string=start_marker+cmd_type

        list_length=len(data_list)

        if list_length <= 0:
            cmd_string += ","  # Just add a comma to separate the header when parsing the data on client side

        else:
            for i in range(list_length):
                cmd_string=cmd_string+","+str(data_list[i])

        return cmd_string+end_marker

    # Configuration file
    def make_config_file(self):  # Function to make a configuration file if it doesn't exist
        print("Making configuration file")

        # Make the configuration file
        self.config['Connection']={
            'host':'localhost',
            'data_port':8090,
            'video_port':8089
        }

        self.config['Video_feed']={
            'resolution':0,
            'fps':10,
            'color':0,
            'feed_type':0,
            'camera_wgt':50.0,
            'thermal_wgt':50.0,
            'enable_thermal':0,
            'thermal_scale':1,
            'thermal_vertical_pos':0,
            'thermal_horizontal_pos':0,
            'enable_hud':1,
        }

        self.config['Video_rec']={
            'vid_file_name':self.video_file_name,
            'vid_file_path':self.video_file_path,
            'vid_rec_res':self.rec_res_index,
            'vid_rec_fps':self.rec_fps,
            'use_cap_res':self.use_cap_resolution,
            'use_cap_fps':self.use_cap_fps,
        }

        self.config['Training_rec']={
            'train_file_name':self.train_file_name,
            'train_file_path':self.train_file_path,
            'train_frame_width':self.train_frame_width,
            'train_frame_height':self.train_frame_height,
            'save_per_every':self.save_per_every,
        }

        self.config['Ctrl_key_cfg']={
            # The disabled keys
            'forward':int(self.disabled_button_press_dict['FORWARD']),
            'backward':int(self.disabled_button_press_dict['BACKWARD']),
            'left':int(self.disabled_button_press_dict['LEFT']),
            'right':int(self.disabled_button_press_dict['RIGHT']),
            'l_turn_f':int(self.disabled_button_press_dict['LEFT_TURN_FORWARD']),
            'r_turn_f':int(self.disabled_button_press_dict['RIGHT_TURN_FORWARD']),
            'l_turn_b':int(self.disabled_button_press_dict['LEFT_TURN_BACKWARD']),
            'r_turn_b':int(self.disabled_button_press_dict['RIGHT_TURN_BACKWARD']),
            'accel':int(self.disabled_button_press_dict['ACCELERATION']),
            'brake':int(self.disabled_button_press_dict['BRAKE']),

            # Order of the key in the array
            'forward_ord':self.move_button_press_order_dict['FORWARD'],
            'backward_ord':self.move_button_press_order_dict['BACKWARD'],
            'left_ord':self.move_button_press_order_dict['LEFT'],
            'right_ord':self.move_button_press_order_dict['RIGHT'],
            'l_turn_f_ord':self.move_button_press_order_dict['LEFT_TURN_FORWARD'],
            'r_turn_f_ord':self.move_button_press_order_dict['RIGHT_TURN_FORWARD'],
            'l_turn_b_ord':self.move_button_press_order_dict['LEFT_TURN_BACKWARD'],
            'r_turn_b_ord':self.move_button_press_order_dict['RIGHT_TURN_BACKWARD'],
            'accel_ord':self.move_button_press_order_dict['ACCELERATION'],
            'brake_ord':self.move_button_press_order_dict['BRAKE'],

            # Send command disabled
            'disable_send_command':int(self.disable_send_command),
        }

        self.config['Pan_tilt_cfg']={
            'pan_left':self.pan_tilt_limit_dict['PAN_LEFT'],
            'pan_right':self.pan_tilt_limit_dict['PAN_RIGHT'],
            'tilt_up':self.pan_tilt_limit_dict['TILT_UP'],
            'tilt_down':self.pan_tilt_limit_dict['TILT_DOWN'],
            'pan_scaler':self.pan_scaler,
            'tilt_scaler':self.tilt_scaler,
            'pan_tilt_enable':int(self.enable_mouse_pan_tilt),
        }

        self.config['Gas_thresh']={
            'eco2':500,
            'tvoc':4000
        }

        self.config['Particle_thresh']={
            'red':1000,
            'green':2000,
            'ir':3000
        }

        self.config['SMS_alert']={
            'alert_number':"09876543210",
            'alert_enable':0
        }

        # Write the configuration file
        self.write_to_config_file()

    def write_to_config_file(self):
        with open(self.config_path, 'w') as configfile:
            self.config.write(configfile)

    def update_defaults_from_config(self):
        # For the connection toolbox
        self.get_connection_settings_cfg()
        self.update_connection_info_bar()  # Display what host and ports are currently being used
        self.update_host_and_ports_input_section()

        # For the video feed toolbox
        self.get_video_feed_setting_cfg()
        self.get_video_disp_settings_cfg()

        # For the video recording toolbox
        self.get_video_recording_setting_cfg()

        # For the training data recording toolbox
        self.get_training_recording_setting_cfg()

        # For the disabled key and order setting section
        self.get_disabled_key_settings_cfg()

        # For the pan and tilt settings toolbox
        self.get_pan_tilt_settings_cfg()

        # For the gas sensor thresholds of the sensor settings toolbox
        self.get_gas_threshold_setting_cfg()

        # For the particles sensor thresholds of the sensor setting toolbox
        self.get_particle_threshold_settings_cfg()

        # For the SMS alert settings in the toolbox
        self.get_sms_setting_cfg()

    # Process main menu triggers
    def process_menu_trigger(self,q_action_obj):
        if q_action_obj.text() == "Exit":
            self.show_exit_message()

        elif q_action_obj.text() == "Reboot Raspberry Pi":
            self.reboot_rpi_message()

        elif q_action_obj.text() == "Reset Microcontroller":
            self.reset_uc_message()

        elif q_action_obj.text() == "Power Off Raspberry Pi":
            self.power_off_rpi()

    def reboot_rpi_message(self):  # Function to reboot the RPi
        msg=QtGui.QMessageBox()

        reboot_btn_msg=QtGui.QPushButton('Reboot')

        msg.setIcon(QtGui.QMessageBox.Question)
        msg.setWindowIcon(QtGui.QIcon("bot_icon.jpg"))
        msg.setWindowTitle("Reboot Raspberry Pi")
        msg.setText("Are you sure you want to reboot the Raspberry Pi?")

        msg.setInformativeText("The connection to the robot will be lost and reconnection will be required.")

        msg.setStandardButtons(QtGui.QMessageBox.Cancel)
        msg.addButton(reboot_btn_msg,QtGui.QMessageBox.YesRole)
        msg.setDefaultButton(QtGui.QMessageBox.Cancel)

        msg.exec_()  # The return value from executing the above message but return value is not used

        if msg.clickedButton() == reboot_btn_msg:
            print("Rebooting RPi")
            cmd=self.format_command(self.socket_send_header_dict['REBOOT_RPI'],[])

            self.send_socket_commands(cmd)

        else:
            print("Rebooting RPi cancelled")
            pass

    def reset_uc_message(self):  # Function to reset the arduino
        msg = QtGui.QMessageBox()

        reset_btn_msg = QtGui.QPushButton('Reset')

        msg.setIcon(QtGui.QMessageBox.Question)
        msg.setWindowIcon(QtGui.QIcon("bot_icon.jpg"))
        msg.setWindowTitle("Reset Microcontroller")
        msg.setText("Are you sure you want to reset the microcontroller?")

        msg.setInformativeText("The robot's movement will be disabled for some about of time as the reset completes.")

        msg.setStandardButtons(QtGui.QMessageBox.Cancel)
        msg.addButton(reset_btn_msg, QtGui.QMessageBox.YesRole)
        msg.setDefaultButton(QtGui.QMessageBox.Cancel)

        msg.exec_()  # The return value from executing the above message but return value is not used

        if msg.clickedButton() == reset_btn_msg:
            print("Resetting microcontroller")
            cmd=self.format_command(self.socket_send_header_dict['RESET_UC'],[])

            self.send_socket_commands(cmd)

        else:
            print("Resetting microcontroller cancelled")
            pass

    def power_off_rpi(self):  # Function to power off the RPi
        msg = QtGui.QMessageBox()

        pwr_btn_msg = QtGui.QPushButton('Power Off')

        msg.setIcon(QtGui.QMessageBox.Warning)
        msg.setWindowIcon(QtGui.QIcon("bot_icon.jpg"))
        msg.setWindowTitle("Power Off Raspberry Pi")
        msg.setText("Are you sure you want to power off the Raspberry Pi?")

        msg.setInformativeText("The Raspberry Pi cannot be restarted and has to be manually turned on again.")

        msg.setStandardButtons(QtGui.QMessageBox.Cancel)
        msg.addButton(pwr_btn_msg, QtGui.QMessageBox.YesRole)
        msg.setDefaultButton(QtGui.QMessageBox.Cancel)

        msg.exec_()  # The return value from executing the above message but return value is not used

        if msg.clickedButton() == pwr_btn_msg:
            print("Powering off Raspberry Pi")
            cmd=self.format_command(self.socket_send_header_dict['PWR_OFF_RPI'],[])

            self.send_socket_commands(cmd)

        else:
            print("Power off cancelled")
            pass

    # Process help menu triggers
    def process_help_trigger(self,q_action_obj):
        if q_action_obj.text() == "About":
            self.show_about_dialog()

    @staticmethod
    def show_about_dialog():  # Show the about dialog :)
        from About_UI import Ui_AboutDialog
        about_dialog=QtGui.QDialog()  # Object of the dialog
        about_ui=Ui_AboutDialog()   # Object of the about UI
        about_ui.setupUi(about_dialog)  # Setup the about UI
        about_ui.buttonBox.accepted.connect(about_dialog.close)  # Connect the dialog okay button to closing the dialog
        about_dialog.show()  # Show the dialog box
        about_dialog.exec_()  # Return value

    # Wifi status update function
    def update_wifi_status(self):
        self.ui.wifiQualityLabel.setText(str(self.wifi_link_quality))
        self.ui.wifiSignalLevelLabel.setText(str(self.wifi_signal_level))

    # Closing app functions
    def closeEvent(self, event):  # Override the normal closing function
        self.show_exit_message()
        event.ignore()  # If the user cancelled application closing ignore the event

    def show_exit_message(self):  # Function get called in the close event
        msg=QtGui.QMessageBox()

        msg.setIcon(QtGui.QMessageBox.Warning)
        msg.setWindowIcon(QtGui.QIcon("bot_icon.jpg"))
        msg.setWindowTitle("Exit")
        msg.setText("Are you sure you want to exit "+self.app_name+" ?")

        msg.setInformativeText("The connection to the robot will be terminated.")

        msg.setStandardButtons(QtGui.QMessageBox.Yes | QtGui.QMessageBox.Cancel)
        msg.setDefaultButton(QtGui.QMessageBox.Cancel)

        retval=msg.exec_()  # The return value contains which button was was pressed

        if retval == QtGui.QMessageBox.Yes:
            print("Application exiting")
            # The thread for the socket image and data can be stopped here but since the thread is daemon it not needed

            if self.enable_video_recording:
                self.vid_saver.close_video_file()  # If video recording was enabled close the file before exit

            if self.enable_training_data_recording:
                self.train_data_saver.close_training_data_file()  # Save the training data if recording was enabled

            sys.exit()  # Finally exit the GUI application
        else:
            print("Application exit cancelled")
            pass


if __name__ == '__main__':
    app=QtGui.QApplication(sys.argv)
    window=MainWindow()
    window.show()
    sys.exit(app.exec_())