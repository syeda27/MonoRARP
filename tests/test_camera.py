import cv2
import time

# Set resolution
cap = cv2.VideoCapture(1)

def set_res(cap, x,y):
    input =  (cap.get(cv2.CAP_PROP_FRAME_WIDTH),
              cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(x))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(y))
    output = (cap.get(cv2.CAP_PROP_FRAME_WIDTH),
              cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if output == (x, y):
        return (str(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                str(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    return ""

resolutions = [(640, 480), (1024, 768), (1028, 800), (1280,1024),
               (1080,1200), (1440, 1080), (1920, 1080), (2048, 1080),
               (2160, 1080), (1280, 720), (1152, 720), (960, 720)]

for x,y in resolutions:
    out = set_res(cap, x, y)
    if out != "":
        print(out)
