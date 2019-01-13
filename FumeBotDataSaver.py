
"""
FUMEBOT DATA SAVER

This is a library that contains all the function to save the video data
coming from the robot into a video file as well as to create training
data for training the neural network. Sensor data are saved along with the
video file since the HUD shows some of the sensor data as well. Contains
a class for each of the above mentioned tasks. A separate class can be used
to view the generated training data.

Written by  :  Ajith Thomas
Date        :  29-5-2018
"""

import os
import cv2
import numpy as np
import datetime
import threading
from PyQt4 import QtCore


class FumeBotVideoSaver:  # For saving the video

    def __init__(self,name='Video.avi',path='~\\Documents\\FumeBot\\Videos',width=1280,height=720,fps=20):

        self.vid_file_name=name
        self.vid_path=os.path.expanduser(path)
        self.width=width
        self.height=height
        self.fps=fps

        self.frame_count=0

        self.vid_path_to_file= self.vid_path + '\\' + self.vid_file_name  # This is the final video file path
        self.path_to_file_exists=False

        try:
            os.makedirs(self.vid_path)
            print("Video directory does not exist, making new directory")
            self.path_to_file_exists=False
        except FileExistsError:
            print("Video storage directory already exists")
            self.path_to_file_exists=True

        self.fourcc=cv2.VideoWriter_fourcc('D','I','V','X')
        self.vid_out=cv2.VideoWriter()

    def get_file_path_status(self):  # Function to say whether the file path had to be created or not and the path
        return self.path_to_file_exists, self.vid_path

    def change_video_save_prop(self,name,path,width,height,fps):  # Function to change video properties

        # Video file is closed to a start a new file with new properties
        self.close_video_file()

        self.vid_file_name=name
        self.vid_path=os.path.expanduser(path)
        self.width=width
        self.height=height
        self.fps=fps

        self.vid_path_to_file = self.vid_path + '\\' + self.vid_file_name  # This is the final video file path
        self.path_to_file_exists = False

        try:
            os.makedirs(self.vid_path)
            self.path_to_file_exists = False
            print("Video storage directory does not exist, making new directory")
        except FileExistsError:
            self.path_to_file_exists = True
            print("Video storage directory already exists")

    def save_frames_to_video_file(self, frame, timestamped=False):  # Function to save frames to a video file

        if not self.vid_out.isOpened():  # This if statement is executed once
            print("Video writer was not opened. Opening...")

            temp_vid_path_to_file=self.vid_path_to_file

            if timestamped is True:
                timestamp_str=datetime.datetime.now().strftime("-%Y-%B-%d-%H-%M-%S")+'.avi'
                temp_vid_path_to_file=self.vid_path_to_file.replace('.avi',timestamp_str)

            retval=self.vid_out.open(temp_vid_path_to_file, self.fourcc, self.fps, (self.width, self.height))

            if retval is True:
                print("Video writer opened")
            else:
                print("Video writer opening failed!")

        if self.vid_out.isOpened():  # Write the video frames to file
            self.vid_out.write(frame)
            self.frame_count += 1
            return True

        else:
            return False

    def close_video_file(self):  # This closes the video writer file and saves the video
        if self.vid_out.isOpened():
            print("Video writer closed")
            self.vid_out.release()
            self.frame_count=0  # Reset the counter to zero

            return True  # If the video is closed properly

        else:
            return False

    def get_frame_count(self):  # Function to get frame count of the recorded frames
        return self.frame_count


class FumeBotTrainingDataSaver:  # To save the training data

    def __init__(self,name='training_data.npy',path='~\\Documents\\FumeBot\\Training',saves_ps=100):

        self.train_file_name=name
        self.train_path=os.path.expanduser(path)
        self.training_data=[]
        self.saves_per_every=saves_ps  # Save the training data per every X amount of data

        self.train_path_to_file=self.train_path+'\\'+self.train_file_name  # Final path to the file

        self.path_to_file_exists=False

        self.training_file_loaded=False  # To say whether a file has already been loaded or not

        # For the meta file
        self.meta_file_name=self.train_file_name.replace(".npy",".meta")
        self.meta_file_path=self.train_path
        self.meta_path_to_file=self.meta_file_path+'\\'+self.meta_file_name

        self.meta_file=None

        self.meta_file_available=False

        self.key_order_from_meta={}
        self.disabled_key_from_meta={}

        try:
            os.makedirs(self.train_path)
            print("Training data storage directory does not exist, making new directory")
            self.path_to_file_exists=False
        except FileExistsError:
            print("Training data storage directory already exists")
            self.path_to_file_exists=True

    def get_path_to_file_status(self):  # To check whether the file had to be created or not
        return self.path_to_file_exists,self.train_path

    def change_train_save_prop(self,name,path,saves_ps):  # Function to change the properties of the file
        """
        Call this function when changing any of the properties of the file.
        Do not call the init method of this class as different functionality might be
        added later. Also this function saves the current loaded file and then changes
        the file properties. Should be called only once.
        """
        if self.training_file_loaded:  # If a file is already loaded save that file before changing properties
            print("Training file already loaded, saving current file")
            np.save(self.train_path_to_file,self.training_data)
            print("Training file saved, changing properties")

        self.train_file_name = name
        self.train_path = os.path.expanduser(path)
        self.training_data = []
        self.saves_per_every = saves_ps  # Save the training data per every X amount of data

        self.train_path_to_file = self.train_path + '\\' + self.train_file_name  # Final path to the file

        self.path_to_file_exists = False

        self.training_file_loaded = False  # To say whether a file has already been loaded or not

        try:
            os.makedirs(self.train_path)
            print("Training data storage directory does not exist, making new directory")
            self.path_to_file_exists = False
        except FileExistsError:
            print("Training data storage directory already exists")
            self.path_to_file_exists = True

    def open_training_file(self,file_name,file_path):  # Function to open and load the training file

        """
        This function should be called only once as it checks whether
        a given training file exists otherwise a new file will be created
        in the save training data function.
        """

        self.train_file_name=file_name
        self.train_path=os.path.expanduser(file_path)
        self.training_data=[]

        self.train_path_to_file=self.train_path+'\\'+self.train_file_name

        if os.path.isfile(self.train_path_to_file):
            print("File is available, loading previous training data")
            self.training_data=list(np.load(self.train_path_to_file))
            self.training_file_loaded=True
        else:
            print("File not available, new file will be created")
            self.training_data=[]  # Reset the training data if file is not found
            self.training_file_loaded=False

        return self.training_file_loaded, len(self.training_data)

    def save_training_data(self,train_input,train_label):  # Function to save data continuously

        """
        This function can be called multiple times to save the training input
        and label. If a given file was found by the open training file function,
        this function appends new data to the existing training file otherwise
        a new file is created.
        """

        self.training_data.append([train_input,train_label])

        save_done=False

        if len(self.training_data) == self.saves_per_every:
            np.save(self.train_path_to_file,self.training_data)
            save_done=True

        return save_done,len(self.training_data)  # Return whether file was saved and size of the training file

    def close_training_data_file(self):  # Function to save the training data if data recording was saved
        np.save(self.train_path_to_file,self.training_data)

    # For the meta file
    def open_meta_file_read_only(self,file_name,file_path):  # Function to open a file in read only mode
        self.meta_file_name=file_name.replace(".npy",".meta")
        self.meta_file_path=os.path.expanduser(file_path)

        self.meta_path_to_file=self.meta_file_path+'\\'+self.meta_file_name

        self.meta_file_available=False

        if os.path.isfile(self.meta_path_to_file):
            self.meta_file_available=True
            self.meta_file=open(self.meta_path_to_file,'r')  # Opened in read mode so is not accidentally overwritten
            print("Meta file exists for the selected numpy file")
        else:
            print("Meta file not available")
            self.meta_file_available=False

        return self.meta_file_available

    def open_meta_file_write_only(self,file_name,file_path):  # Function to open a file in write only mode
        if self.open_meta_file_read_only(file_name,file_path) is False:
            self.meta_file=open(self.meta_path_to_file,'w')

    def write_cfg_to_meta_file(self,paren_file_name,key_order_dict,disable_key_dict):  # Write to file
        self.meta_file.write(str(paren_file_name)+"\n")
        self.meta_file.write(self.dict_to_string(key_order_dict))  # 1st one
        self.meta_file.write(self.dict_to_string(disable_key_dict))  # 2nd one

        self.close_meta_file()  # Close the file automatically

    def read_cfg_from_meta_file(self):  # Read from file
        name=self.meta_file.readline()    # File name string
        ko_str=self.meta_file.readline()  # Key order string
        dk_str=self.meta_file.readline()  # Disabled key string

        self.close_meta_file()  # Close the file automatically

        self.key_order_from_meta=self.convert_dict_value_to_int(self.string_to_dict(inp_str=ko_str))
        self.disabled_key_from_meta=self.convert_dict_value_to_bool(self.string_to_dict(inp_str=dk_str))

        return name, self.key_order_from_meta,self.disabled_key_from_meta

    @staticmethod
    def compare_dicts(dict_1,dict_2):  # Function to compare dictionaries
        if dict_1 == dict_2:
            return True
        else:
            return False

    def close_meta_file(self):  # Function to close an opened meta file
        self.meta_file.close()

    @staticmethod
    def string_to_dict(inp_str):  # Function to create a dict to string
        inp_str = str(inp_str).strip()
        output_dict = dict(item.split("=") for item in inp_str.split(";"))

        return output_dict

    @staticmethod
    def convert_dict_value_to_int(inp_dict):  # Function to cast the values of dict to int
        for key, value in inp_dict.items():
            inp_dict[key] = int(value)

        return inp_dict

    @staticmethod
    def convert_dict_value_to_bool(inp_dict):  # Function to cast the values of dict to bool
        for key, value in inp_dict.items():
            if value == 'True':
                inp_dict[key] = True
            else:
                inp_dict[key] = False

        return inp_dict

    @staticmethod
    def dict_to_string(inp_dict):  # Function to create the string to write to file from a dict
        string_to_file = ""
        i = 0
        for key, value in inp_dict.items():
            string_to_file = string_to_file + (str(key) + "=" + str(value))
            i += 1
            if i == len(inp_dict):
                break
            else:
                string_to_file = string_to_file + ";"

        return string_to_file + "\n"


class ViewTrainingData:  # For viewing the training data
    """
    This class is used for viewing the collected training data
    in the GUI application in a window which is not implemented in the App.
    """

    newFrame = QtCore.pyqtSignal(np.ndarray,np.ndarray)

    def __init__(self,file_name,file_path):

        self.view_file_name=file_name
        self.view_file_path=os.path.expanduser(file_path)

        self.view_path_to_file=self.view_file_path+"\\"+self.view_file_name

        self.view_thread = threading.Thread(target=self.get_training_data)
        self.view_thread.setDaemon(True)
        self.view_thread_run = False

        self.training_data=np.ndarray

        self.view_completed_once=False

    def start_view_thread(self):  # Function to start the view thread
        if not self.view_thread.isAlive():
            self.view_thread_run=True
            self.view_thread.start()

    def stop_view_thread(self):  # Function to stop the view thread
        if self.view_thread.isAlive():
            self.view_thread_run=False

    def get_training_data(self):  # Function is threaded
        print("View thread started")

        self.training_data=np.load(self.view_path_to_file)

        while self.view_thread_run is True:

            if not self.view_completed_once:

                for data in self.training_data:
                    frame_train_input=data[0]
                    key_label=data[1]

                    if self.view_thread_run is True:
                        break

                    self.newFrame.emit(frame_train_input,key_label)

                self.view_completed_once=True

        print("View thread exited")
























