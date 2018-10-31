"""
A file to declare MultiTracker wrapper classes.
These classes must conform to the API in the abstract class, but can be defined
for extension multiobject trackers.
"""

import cv2
from driver_risk_utils import general_utils, tracker_utils


class MultiTrackerWrapper:
    """
    Default to handling OpenCV multi_tracker with KCF.
    Inherit the class to use
    """
    def __init__(self, type):
        self.tracker_type = type
        self.mult_tracker = None

    def initialize_tracker(self, image, box):
        """
        Main API call #1 from tracker.py
        """
        pass

    def update_all(self, image, box=None):
        """
        Main API call #2 from tracker.py
        Must return:
          Ok or not
          Boxes for current image
        """
        pass

    def _add_one_tracker(self, image, box):
        pass


class OpenCVMultiTracker(MultiTrackerWrapper):
    def __init__(self, type):
        if type == "KCF":
            self.single_tracker_fnc = cv2.TrackerKCF_create
        else:
            tracker_utils.raise_undefined_tracker_type(type)
        self.tracker_type = type
        self.multi_tracker = cv2.MultiTracker_create()


    def initalize_tracker(self, image, boxes):
        for box in boxes:
            self._add_one_tracker(image, box)

    def update_all(self, image, box, verbose=False):
        ok, boxes = self.multi_tracker.update(image)
        if verbose:
            print("shape:", image.shape)
            print("val", image)
            print(ok)
        return ok, boxes

    def _add_one_tracker(self, image, box):
        im_height, im_width, _ = image.shape
        self.multi_tracker.add(
            self.single_tracker_fnc(),
            image,
            general_utils.convert(im_height, im_width, box)
        )
