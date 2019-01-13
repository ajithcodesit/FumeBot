import cv2
import numpy as np
import pandas as pd
from collections import Counter
from random import shuffle

# Provide the correct path to the dataset
train_data=np.load("C:\\Users\\Ajith Thomas\\Documents\\FumeBot\\Training\\training_data_obst_crs.npy")

df=pd.DataFrame(train_data)  # The data frame

print("Dataset count for different movement")
print(Counter(df[1].apply(str)))

# Collect all the forwards, lefts and rights
forwards=[]
lefts=[]
rights=[]

shuffle(train_data)

# Sort the direction key presses to the respective lists
for data in train_data:
    frame=data[0]
    label=data[1]

    frame=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    if label == [1,0,0]:
        lefts.append([frame,label])

    elif label == [0,1,0]:
        forwards.append([frame,label])

    elif label == [0,0,1]:
        rights.append([frame,label])
    else:
        print('No matches found!')

forwards=forwards[:len(lefts)][:len(rights)]  # Splitting the labels according to the number of lefts and rights
lefts=lefts[:len(forwards)]  # Change the number of lefts according to the new forwards
rights=rights[:len(forwards)]  # Change the number of rights according to the new forwards

final_data=forwards+lefts+rights  # Concatenate all the lists
shuffle(final_data)  # A final shuffle before saving

print("Final data size = "+str(len(final_data)))

print("Saving training data...")

np.save('C:\\Users\\Ajith Thomas\\Documents\\FumeBot - Training Datasets\\training_data_obs_crs_v1.npy',final_data)

print("Training data saved")
