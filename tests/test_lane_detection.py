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

DIR_WITH_FRAMES = "Hwy101_frames"
start = time.time()
num_images = 0

LANE_DETECTOR = lane_detector.LaneDetector()
for image_file in os.listdir(DIR_WITH_FRAMES):
    print(os.path.join(DIR_WITH_FRAMES, image_file))
    image = cv2.imread(os.path.join(DIR_WITH_FRAMES, image_file)) # (in color)
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
