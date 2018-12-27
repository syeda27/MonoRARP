import cv2
import numpy as np

cap = cv2.VideoCapture(1)

resolution = (1920, 1080) # [(640 x 480), (1280 x 720), (1920 x 1080)]
cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])

fourcc = cv2.VideoWriter_fourcc(*'MP4V')
writer = cv2.VideoWriter(
    'test_'+str(resolution[1])+'.mp4',
    fourcc,
    10,
    resolution)
print("creating writer of image shape (w,h):", resolution)

for i in range(50):
    ret, frame = cap.read()
    print("writing image of shape:", frame.shape)
    writer.write(frame)
cap.release()
writer.release()
