from keras.models import Sequential
from keras.layers import Dense,Dropout,Flatten
from keras.layers import Conv2D,MaxPool2D,ZeroPadding2D
from keras.optimizers import Adam

'''
**Change Log**
- Removed SGD and used Adam optimizer
- Removed batch normalization layers
- Tried to used ReLU at the FC layers did not yield good results
- Batch normalization removal has caused the model to produce consistent results
'''


def AlexNet(width,height,lr):
    model = Sequential()

    # Input layer is defined in the first convolutional layer (frames of WxH with 1 color channel)
    model.add(Conv2D(filters=96, kernel_size=(11, 11), strides=(4, 4), activation='relu', input_shape=(width, height, 1)))
    model.add(MaxPool2D(pool_size=(3, 3), strides=(2, 2)))
    model.add(ZeroPadding2D(padding=(1, 1)))

    model.add(Conv2D(filters=256, kernel_size=(5, 5), strides=(1, 1), activation='relu'))
    model.add(MaxPool2D(pool_size=(3, 3), strides=(2, 2)))
    model.add(ZeroPadding2D(padding=(1, 1)))

    model.add(Conv2D(filters=384, kernel_size=(3, 3), activation='relu'))
    model.add(ZeroPadding2D(padding=(1, 1)))
    model.add(Conv2D(filters=384, kernel_size=(3, 3), activation='relu'))
    model.add(ZeroPadding2D(padding=(1, 1)))
    model.add(Conv2D(filters=256, kernel_size=(3, 3), activation='relu'))
    model.add(ZeroPadding2D(padding=(1, 1)))
    model.add(MaxPool2D(pool_size=(3, 3), strides=(2, 2)))

    model.add(Flatten())
    model.add(Dense(units=4096, activation='tanh'))
    model.add(Dropout(rate=0.5))

    model.add(Dense(units=4096, activation='tanh'))
    model.add(Dropout(rate=0.5))

    # Final output layer
    model.add(Dense(units=3, activation='softmax'))

    # Adam optimizer
    optimizer = Adam(lr=lr)
    model.compile(loss='categorical_crossentropy', optimizer=optimizer, metrics=['accuracy'])

    print(model.summary())

    return model


