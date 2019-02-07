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

    # TODO display object info with units!! Currently ego speed in mph,
    # everything else in metric...
    def display_info(self,
                     state,
                     risk,
                     speed,
                     boxes_with_labels,
                     frame_time=None,
                     show_global_stats=True,
                     rel_horizon=0.5,
                     show_speed=True):
        i = 0
        for object_key, (b, label) in sorted(boxes_with_labels.items()):
            aspect_ratio_off = general_utils.check_aspect_ratio(b)
            if label != "car" or aspect_ratio_off:
                display_utils.make_rectangle(self.imgcv,
                               b,
                               self.d_args.invalid_color,
                               self.d_args.thick)
                continue
            text = display_utils.make_text(str(object_key), state[object_key], frame_time)
            # object id on box:
            display_utils.outline_object_text(text, self.imgcv, self.d_args, i)
            display_utils.outline_rectangle(self.imgcv, b, self.d_args, object_key)
            i += 1
        if show_global_stats:
            if not show_speed: speed = None
            display_utils.outline_global_text(self.imgcv, risk, speed, self.d_args)
        #self.imgcv = cv2.resize(self.imgcv,(1280,720)) #[(640 x 480), (1280 x 720), (1920 x 1080)]
        display_utils.mark_center_lines(self.imgcv, self.d_args)
        display_utils.mark_horizon_line(self.imgcv, self.d_args, rel_horizon)
        return self.imgcv
