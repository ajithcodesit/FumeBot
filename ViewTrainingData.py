import cv2
import numpy as np

training_data=np.load('C:\\Users\\Ajith Thomas\\Documents\\FumeBot - Training Datasets\\training_data_test_obst_crs_v1.5.npy')

font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 1
font_thickness = 1
font_color = (0, 255, 0)

frame_size_printed=False

print("Training data size="+str(len(training_data)))

for data in training_data:

    frame=data[0]
    pressed_key=data[1]

    if frame_size_printed is False:
        print(frame.shape[:2])
        frame_size_printed=True

    frame=cv2.cvtColor(frame,cv2.COLOR_GRAY2BGR)

    frame=cv2.resize(frame,(640,480),interpolation=cv2.INTER_LINEAR)

    cv2.putText(frame, str(pressed_key), (50, 50), font, font_scale, font_color, font_thickness, cv2.LINE_AA)

    cv2.imshow("Training frame", frame)

    if cv2.waitKey(25) & 0xff == ord('q'):
        cv2.destroyAllWindows()
        break

