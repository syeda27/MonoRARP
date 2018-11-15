"""
A file to declare MultiTracker wrapper classes.
These classes must conform to the API in the abstract class, but can be defined
for extension multiobject trackers.
"""

import cv2
from driver_risk_utils import general_utils, tracker_utils
#import particle_tracker as pt # TODO
import particle_tracker_dp as particle_tracker_dp
import particle_trackers_ar as particle_tracker

class MultiTrackerWrapper:
    """
    Default to handling OpenCV multi_tracker with KCF.
    Inherit the class and implement the functions before using.
    """
    def __init__(self):
        raise NotImplementedError("Please implement me!")

    def initialize_tracker(self, image, boxes=None, labels=None):
        """
        Main API call #1 from tracker.py

        Arguments
          image: a np 2D array containing pixel values.
          boxes: list of boxes detected in the image. Can be none.
            Each box is a tuple (left, right, top, bottom)
          labels: list of the corresponding class labels for each box.
        """
        raise NotImplementedError("Please implement me!")

    def update_all(self, image, boxes=None, labels=None):
        """
        Main API call #2 from tracker.py

        Arguments
          image: a np 2D array containing pixel values.
          boxes: list of boxes detected in the image. Can be none.
            Each box is a tuple (left, right, top, bottom)
          labels: list of the corresponding class labels for each box.

        Returns:
          ok: boolean that is false if tracker lost a vehicle.
          boxes_with_labels: dictionary <int : box coordinates>
            object key as the dict key.
            detected object bounding box as the value.
            Will be in absolute pixel value coordinates.
        """
        raise NotImplementedError("Please implement me!")


class OpenCVMultiTrackerWrapper(MultiTrackerWrapper):
    def __init__(self, type):
        if type == "KCF":
            self.single_tracker_fnc = cv2.TrackerKCF_create
        else:
            tracker_utils.raise_undefined_tracker_type(type)
        self.tracker_type = type
        self.multi_tracker = cv2.MultiTracker_create()

    def initialize_tracker(self, image, boxes, labels=None):
        im_height, im_width, _ = image.shape
        for box in boxes:
            self.multi_tracker.add(
                self.single_tracker_fnc(),
                image,
                general_utils.convert(im_height, im_width, box)
            )

    def update_all(self, image, boxes=None, labels=None, verbose=False):
        ok, boxes = self.multi_tracker.update(image)
        if verbose:
            print("shape:", image.shape)
            print("val", image)
            print(ok)
        box_with_labels = dict()
        for i,b in enumerate(boxes):
            box_with_labels[i] = (b, labels[i])
        return ok, box_with_labels


class ParticleTrackerWrapper(MultiTrackerWrapper):
    def __init__(self):
        #self.multi_tracker = particle_tracker.ParticleTracker(10, 10)
        self.multi_tracker = particle_tracker_dp.ParticleTrackerDP(10, 10)

    def initialize_tracker(self, image, boxes=None, labels=None):
        # remove_and_call_again?
        self.multi_tracker.reset_all_trackers()
        self.multi_tracker.update_all(image, boxes, labels)

    def update_all(self, image, boxes=None, labels=None, verbose=False):
        boxes_with_labels = self.multi_tracker.update_all(image, boxes, labels, verbose)
        im_h, im_w, _ = image.shape
        for obj_key, (b, label) in boxes_with_labels.items():
            boxes_with_labels[obj_key] = (general_utils.convert(im_h, im_w, b), label)
        return True, boxes_with_labels
        # returns ok, boxes
        #raise NotImplementedError("Please implement me!")
