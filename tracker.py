"""
This file will hold a tracker class, which itself will call some other trackers,
such as the OpenCV or special trackers.
It is essentially a wrapper class to make the main file have consistent API
"""

import cv2
import numpy as np
from collections import defaultdict
from driver_risk_utils import general_utils, tracker_utils
import multi_trackers

class Tracker:
    """
    Tracker class that serves as the interface between our system and openCV or
    other trackers that we have implemented.

    Main function is `update_one`:
        It updates one of the trackers with the output from a network,
            the input image queue, and an index for the image.
    """

    def __init__(self,
                 args,
                 tracker_type,
                 image_height,
                 image_width,
                 category_index):
        """
        Arguents
          args, and argument_utils args object, containing:
            det_thresh
            tracker_refresh
            track
          tracker_type:
            String. What type of tracker?
          image_height:
            Height of the image in pixels. Integer.
          image_width:
            Width of the image in pixels. Integer.
          category_index:
            Dictionary for what categories are which from the model.
        """
        self.tracker_type = tracker_type
        self.det_thresh = args.det_thresh
        self.tracker_refresh = args.tracker_refresh

        self.multi_tracker = None
        self.use_tracker = args.track
        if self.use_tracker:
            self._create_multi_tracker()

        self.init_tracker = True
        self.labels = defaultdict(list) # i -> list of labels
        self.category_index = category_index

    def _create_multi_tracker(self):
        """
        Set the internal self.multi_tracker variable to one of the wrappers
        implemented in multi_trackers.py.
        """
        if self.tracker_type == "KCF":
            self.multi_tracker = multi_trackers.OpenCVMultiTrackerWrapper(
                self.tracker_type
            )
        elif self.tracker_type == "Particle":
            self.multi_tracker = multi_trackers.ParticleTrackerWrapper()
                # TODO update args
        else:
            tracker_utils.raise_undefined_tracker_type(self.tracker_type)

    def update_if_init(self, elapsed_frames):
        """
        Update, based on the elapsed frames and tracker refresh rate initially
        set, our internal flag on if we want to initialize the tracker.
        """
        self.init_tracker = elapsed_frames % self.tracker_refresh == 1

    def should_reset(self):
        """
        Simple function to AND two internal flags. Used externally, for example
        in process_queue()
        """
        return self.use_tracker and self.init_tracker

    def check_and_reset_multitracker(self, state_object):
        """
        If we need to reset the tracker, do so by creating a new multi_tracker
        (which will delete the old one), resetting the label list, and telling
        the referenced state_object to clear its data.
        """
        if self.use_tracker and self.init_tracker:
            self._create_multi_tracker()
            self.lables = defaultdict(list)
            state_object.clear()

    def update_one(self, image_index, net_out, image, verbose=False):
        """
        The main function that is called. Uses the image and object detections
        to update the tracker given a single image.

        Arguments
          image_index: integer
            The index into the object detection network's output, for the
            current image.
          net_out: dict<string: list < boxes, scores, or classes > >
            The output from the object detection network.
          image: np array
            The image.
          verbose: boolean.
            True if we want to print extra logging info.

        Returns:
          boxes: list of (left, right, top, bottom) integer tuples.
            These will be in absolute pixel values
          labels: list < string >
            The list of labels corresponding to the boxes
        """
        boxes = []
        if net_out is not None:
            boxes = net_out['detection_boxes'][image_index][np.where(\
                    net_out['detection_scores'][image_index] >= self.det_thresh)]
        if self.init_tracker:
            self.init_tracker = False
            self.labels = [self.category_index[key]['name'] for key in \
                    net_out['detection_classes'][image_index][np.where(\
                    net_out['detection_scores'][image_index] >= self.det_thresh)]
                    ]
            self.multi_tracker.initialize_tracker(image, boxes)
            im_h, im_w, _ = image.shape
            for i,b in enumerate(boxes):
                boxes[i] = general_utils.convert(im_h, im_w, b)
        else:
            ok, boxes = self.multi_tracker.update_all(image, boxes, verbose)
            if ok is False: # lost tracking
                self.init_tracker = True
        return boxes, self.labels
