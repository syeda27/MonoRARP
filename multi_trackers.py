"""
A file to declare MultiTracker wrapper classes.
These classes must conform to the API in the abstract class, but can be defined
for extension multiobject trackers.
"""

import cv2
from driver_risk_utils import general_utils, tracker_utils
#import particle_tracker as pt # TODO

class MultiTrackerWrapper:
    """
    Default to handling OpenCV multi_tracker with KCF.
    Inherit the class to use
    """
    def __init__(self):
        pass

    def initialize_tracker(self, image, boxes=None):
        """
        Main API call #1 from tracker.py
        """
        pass

    def update_all(self, image, boxes=None):
        """
        Main API call #2 from tracker.py
        Must return:
          Ok or not
          Boxes for current image
        """
        pass


class OpenCVMultiTrackerWrapper(MultiTrackerWrapper):
    def __init__(self, type):
        if type == "KCF":
            self.single_tracker_fnc = cv2.TrackerKCF_create
        else:
            tracker_utils.raise_undefined_tracker_type(type)
        self.tracker_type = type
        self.multi_tracker = cv2.MultiTracker_create()

    def initalize_tracker(self, image, boxes):
        im_height, im_width, _ = image.shape
        for box in boxes:
            self.multi_tracker.add(
                self.single_tracker_fnc(),
                image,
                general_utils.convert(im_height, im_width, box)
            )

    def update_all(self, image, boxes=None, verbose=False):
        ok, boxes = self.multi_tracker.update(image)
        if verbose:
            print("shape:", image.shape)
            print("val", image)
            print(ok)
        return ok, boxes


class ParticleTrackerWrapper(MultiTrackerWrapper):
    def __init__(self):
        #self.multi_tracker = particle_tracker.ParticleTracker(n_p=10,)
        pass

    def initialize_tracker(self, image, boxes=None):
        pass

    def update_all(self, image, boxes=None):
        pass
