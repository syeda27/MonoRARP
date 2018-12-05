"""
LANE DETECTION MAIN

I) IMAGE READING
II) LSD ALGORITHM
III) SCANNING OF THE IMAGE AND DETECTION OF ROAD MARK SIGNATURE
IV) ELIMINATION OF WHITE ROAD MARK DUPLICATES
V) MERGING ALL WHITE ROAD MARKS PREVIOUSLY DETECTED AND GENERATION OF TWO LANES
VI) FILTERING WHITE ROAD MARKS TO REMOVE FALSE DETECTIONS AND CORRECTION OF THE TWO LANES
VII) RECORDING OF CURRENTLY DETECTED LANES
VIII) GENERATION OF LONG TERM AVERAGE OF THE DETECTED LANES
IX) DISPLAY

Author: Juan Carlos Aragon  -  Allstate
Minor Editing: djp42  -  Stanford

See README for more details.

"""

import cv2
import time
import sys
import os

sys.path.append("..")
sys.path.append("../lane_detection_utils") # for calls within
import lane_detector

SOURCE ="/scratch/derek/video_captures/FullFOVandHD/video11a.mp4"

start = time.time()
num_images = 0

height = 720
width = 1280

scan_x_params=(int(width / 4),
               width - int(width / 4),
               int(width / 30))
#scan_x_params=(1480, 2720, 80) # 1280, 2560, 128 breaks it sometimes.
scan_y_params=(int(height / 21),
               int(height / 8),
               int(height / 50))
#scan_y_params=(100, 250, 20) # 108, 432, 72 breaks it sometimes.
scan_window_sz=(int(width / 18), int(height / 20))

scan_x_ini=415
scan_x_end=865
scan_x_step=30
scan_x_params=(scan_x_ini, scan_x_end, scan_x_step)
#inital and end points for the scanning in the y-direction within the image subframe, and step of the scanning
scan_y_ini=150
scan_y_end=300
scan_y_step=20
scan_y_params=(scan_y_ini, scan_y_end, scan_y_step)

#size of the rectangular window used for the scanning
scanning_window_width=80
scanning_window_lenght=100
#scan_window_sz=(120, 160)
subframe_dims=(int(7*height/10), int(17*height/20), 0, width)
# (1350, 1890, 0, 3840) does not work...
# subframe_dims=(1500, 1800, 0, 3840)
subframe_dims=(240,540,0,1280)
print(subframe_dims)

horizontal_tolerance = int(width / 75) # o.g. 50
brightness_ratio_threshold = 1.15
left_margin_detection = int(width / 4)  # o.g. 1500
right_margin_detection = width - int(width / 4) # o.g. 2700
average_window = 6

print(scan_x_params)
print(scan_y_params)
print(scan_window_sz)
print(subframe_dims)
LANE_DETECTOR = lane_detector.LaneDetector(
    scan_x_params, scan_y_params, scan_window_sz, subframe_dims,
    True, # display_lane_lines
    brightness_ratio_threshold,
    horizontal_tolerance,
    left_margin_detection,
    right_margin_detection,
    average_window
)

print(SOURCE)
camera = cv2.VideoCapture(SOURCE)
assert camera.isOpened(), \
        'Cannot capture source'

while camera.isOpened():
    _, image = camera.read()
    LANE_DETECTOR.handle_image(image)
    try:
        print("Official speed: {}".format(LANE_DETECTOR.speed_official))
    except AttributeError:
        print("No official speed.")
    LANE_DETECTOR.display()
    num_images += 1

end_time = time.time()
elapsed_time = end_time - start

print("Lane Detection successfully completed. Please check accuracy.")
print("Test took {} seconds to complete {} frames".format(elapsed_time, num_images))
print("That means an FPS of: {}".format(num_images / elapsed_time))
