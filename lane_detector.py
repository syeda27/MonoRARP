"""
The purpose of this file is to create a usable class out of the lane detection
scripts that Juan Carlos wrote, and integrate it with the DRIVR system.

"""

import cv2
import math
import numpy as np

from lane_detection_utils import *

class LaneDetctor:
    def __init__(self,
                 scan_x_params=(1480, 2720, 80),
                 scan_y_params=(100, 250, 20),
                 scan_window_sz=(120, 160)):
        lane_arg_utils.initialize_lane_detector_members(self)
        #inital and end points for the scanning in the x-direction within the image subframe, and step of the scanning
        self.scan_x_ini, self.scan_x_end, self.scan_x_step = scan_x_params
        #inital and end points for the scanning in the y-direction within the image subframe, and step of the scanning
        self.scan_y_ini, self.scan_y_end, self.scan_y_step = scan_y_params
        #size of the rectangular window used for the scanning
        self.scanning_window_width, self.scanning_window_length = scan_window_sz
