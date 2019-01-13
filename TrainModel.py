import numpy as np
from keras.callbacks import TensorBoard
from DeepNeuralNetworkModel import AlexNet

# Frame information variables
WIDTH=80
HEIGHT=60
LR=1e-4
EPOCH=20

MODEL_PATH='C:\\Users\\Ajith Thomas\\Documents\\FumeBot - DNN Model'
MODEL_NAME='Fumebot-{}-{}-{}-model.json'.format(LR,'AlexNet',EPOCH)
MODEL_SAVE_PATH=MODEL_PATH+"\\"+MODEL_NAME

WEIGHT_PATH='C:\\Users\\Ajith Thomas\\Documents\\FumeBot - DNN Model'
WEIGHT_NAME='Fumebot-{}-{}-{}-weight.h5'.format(LR,'AlexNet',EPOCH)
WEIGHT_SAVE_PATH=WEIGHT_PATH+"\\"+WEIGHT_NAME

CHECKPOINT_FILE_NAME='Model_AlexNet'
CHECKPOINT_PATH='C:\\Users\\Ajith Thomas\\Documents\\FumeBot - DNN Model\\Training Save Files'
CHECKPOINT_FILE_PATH=CHECKPOINT_PATH+'\\'+CHECKPOINT_FILE_NAME

LOG_FILE_NAME='Log'
LOG_PATH='C:\\Users\\Ajith Thomas\\Documents\\FumeBot - DNN Model\\Training Save Files'
LOG_FILE_PATH=LOG_PATH+'\\'+LOG_FILE_NAME

# Path to the training data set
TRAIN_DATA_PATH='C:\\Users\\Ajith Thomas\\Documents\\FumeBot - Training Datasets\\training_data_obs_crs_v1.npy'

train_data=np.load(TRAIN_DATA_PATH)
print("Training data count = "+str(len(train_data))+"\n")

train=train_data[:-500]
test=train_data[-500:]

print("Training data split:")
print("Training Examples = "+str(len(train)))
print("Testing Example = "+str(len(test))+"\n")

train_X=np.array([frame[0] for frame in train]).reshape(-1, WIDTH, HEIGHT, 1)
train_Y=np.array([label[1] for label in train])

test_x=np.array([frame[0] for frame in test]).reshape(-1, WIDTH, HEIGHT, 1)
test_y=np.array([label[1] for label in test])

# Training part
model=AlexNet(WIDTH,HEIGHT,LR)
tensorboard=TensorBoard(log_dir=LOG_FILE_PATH)

model.fit(x=train_X, y=train_Y, batch_size=32, epochs=EPOCH, verbose=2,callbacks=[tensorboard])
score=model.evaluate(x=test_x,y=test_y,batch_size=32,verbose=1)

print(str(model.metrics_names[1])+" = "+str(score[1]*100)+"%")

# Saving the model as a JSON file
print("Saving the model")
model_json=model.to_json()
with open(MODEL_SAVE_PATH,"w") as json_file:
    json_file.write(model_json)
    json_file.close()
print("Model saved")

# Saving the weights
print("Saving the model weights")
model.save_weights(WEIGHT_SAVE_PATH)
print("Weights saved")

# tensorboard --logdir=foo:"C:\Users\Ajith Thomas\Documents\FumeBot - DNN Model\Training Save Files\Log"

