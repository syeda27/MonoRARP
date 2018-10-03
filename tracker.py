'''
This file will hold a tracker class, which itself will call some other trackers,
such as the OpenCV or special trackers.
It is essentially a wrapper class to make the main file have consistent API
'''

import cv2
import numpy as np
from collections import defaultdict
from driver_risk_utils import general_utils


class tracker:
    '''
    Tracker class that serves as the interface between our system and openCV or other
    trackers that we have implemented.

    Initialize with: 
      `args`, that contains:
        det_thresh
        tracker_refresh
        track
      tracker_type:
        Currently just supports "KCF". A string.
      image_height:
        Height of the image in pixels. Integer.
      image_width:
        Width of the image in pixels. Integer.
      category_index:
        Dictionary for what categories are which from the model.

    Main function is `update_one`:
        It updates one of the trackers with the output from a network,
            the input image queue, and an index for the image.
    '''

    def __init__(self,
                 args,
                 tracker_type,
                 image_height,
                 image_width,
                 category_index):
        self.tracker_type = tracker_type
        self.im_height = image_height
        self.im_width = image_width
        self.det_thresh = args.det_thresh
        self.tracker_refresh = args.tracker_refresh

        self.multi_tracker = None
        self.use_tracker = args.track
        if self.use_tracker:
            self.create_multi_tracker()

        self.init_tracker = True
        self.labels = defaultdict(list) # i -> list of labels
        self.category_index = category_index

    def create_multi_tracker(self):
        if self.tracker_type == "KCF":
            self.multi_tracker = cv2.MultiTracker_create()
        else:
            self.raise_undefined_tracker_type()

    def update_if_init(self, elapsed_frames):
        self.init_tracker = elapsed_frames % self.tracker_refresh == 1

    def update_im_shape(self, height, width):
        self.im_height = height
        self.im_width = width

    def should_reset(self):
        return self.use_tracker and self.init_tracker

    def check_and_reset_multitracker(self, state_object):
        if self.should_reset():
            self.create_multi_tracker()
            self.lables = defaultdict(list)
            state_object.clear()

    def update_one(self, image_index, net_out, buffer_input): 
        do_convert = True
        if self.init_tracker:
            self.init_tracker = False
            boxes = net_out['detection_boxes'][image_index][np.where(\
                    net_out['detection_scores'][image_index] >= self.det_thresh)]
            self.labels = [self.category_index[key]['name'] for key in \
                    net_out['detection_classes'][image_index][np.where(\
                    net_out['detection_scores'][image_index] >= self.det_thresh)]
                    ]
            for box in boxes:
                if self.tracker_type == "KCF":
                    self.multi_tracker.add(
                        cv2.TrackerKCF_create(), 
                        buffer_input[image_index],
                        general_utils.convert(self.im_height, self.im_width, box)
                    )
                else:
                    self.raise_undefined_tracker_type()
        else:
            do_convert = False
            ok, boxes = self.multi_tracker.update(buffer_input[image_index])
            '''
            print("shape:", buffer_inp[image_index].shape)
            print("val", buffer_inp[image_index])
            print("len", len(buffer_inp), " ", i)
            print(ok)
            '''
            if ok is False: # lost tracking
                self.init_tracker = True
        return boxes, do_convert, self.labels

    def raise_undefined_tracker_type(self):
        raise ValueError("Invalid tracker type: {}".format(self.tracker_type))
