"""
This file is responsible for the display elements of the DRIVR system.
"""

import numpy as np
import cv2

from driver_risk_utils import display_utils, general_utils

class Display:
    def __init__(self):
        self.d_args = display_utils.DisplayArgs()
        self.imgcv = None

    def update_image(self, im):
        if type(im) is not np.ndarray:
            self.imgcv = cv2.imread(im)
        else:
            self.imgcv = im
        self.d_args.set_im(self.imgcv)

    def display_info(self,
                     state,
                     risk,
                     speed,
                     boxes,
                     labels,
                     fps=6.0,
                     frame_time=None):
        for i, b in enumerate(boxes):
            aspect_ratio_off = general_utils.check_aspect_ratio(b)
            if labels[i] != "car" or aspect_ratio_off:
                display_utils.make_rectangle(self.imgcv,
                               b,
                               self.d_args.invalid_color,
                               self.d_args.thick)
                continue
            text = display_utils.make_text(str(i), state[i], frame_time)
            # object id on box:
            display_utils.outline_object_text(text, self.imgcv, self.d_args, i)
            display_utils.outline_rectangle(self.imgcv, b, self.d_args)
        display_utils.outline_global_text(self.imgcv, risk, speed, self.d_args)
        return self.imgcv
