"""
This file contains some general utility functions used in the driver risk
estimation system.
"""

import time
import numpy as np
from collections import defaultdict

# TODO:
"""
class box:
    def __init__(self, box):
        self.left, self.right, self.top, self.bot = b

    def get_box():
        return (self.left, self.right, self.top, self.bot)
"""

def get_fps(start, frames):
    """
    Simple wrapper to computer frames per second.
    If 0 or 1 frames recorded, return 1.

    Arguments:
      start: float
        the time that the process started, in seconds.
      frames: int
        the number of frames that have been processed so far.

    Returns:
      fps: float
        The number of frames that have been processed per second.
    """
    if frames <= 1:
        return 1
    elapsed_time = time.time() - start
    return frames / elapsed_time

def filter_boxes(net_out, threshold, horizon, cat_index, i, keep_classes={"car"}):
    print(net_out['detection_scores'][i])
    indexes = np.where(\
            net_out['detection_scores'][i] >= threshold \
            and cat_index[
                net_out['detection_classes'][i]
            ]['name'] in keep_classes \
            and net_out['detection_boxes'][i][3] < horizon)
    boxes = net_out['detection_boxes'][i][indexes]
    labels = [cat_index[key]['name'] for key in \
            net_out['detection_classes'][i][indexes]]
    return boxes, labels


def check_aspect_ratio(box, height_ratio=5, width_ratio=3):
    """
    Check that a box satisfies the aspect ratio constraints:

    Arguments:
      box: (int, int, int, int)
        (left, right, top, bot) box coordinates, can be normalized or absolute.
      height_ratio: float
        We make sure width <= height_ratio * height
      width_ratio: float
        We make sure height <= width_ratio * width

    Returns:
      boolean, whether or not both constraints are satisfied.
    """
    (left, right, top, bot) = box
    width = right - left
    height = bot - top
    return width > height_ratio * height or height > width_ratio * width

# ymin, xmin, ymax, xmax  ===> left, right, top, bot
def convert(im_height, im_width, box):
    """
    Converts the box from normalized coordinates to absolute pixel values.

    Arguments:
      im_height: int
        The image height, in pixels
      im_width: int
        The image width, in pixels
      box: (int, int, int, int)
        (left, right, top, bot) box coordinates, normalized (0 <= x <= 1).

    Returns:
      box: (int, int, int, int)
        (left, right, top, bot) == (ymin, xmin, ymax, xmax)
        box coordinates, absolute (0 <= x <= image_dim).
    """
    return (int(box[1] * im_width),
            int(box[3] * im_width),
            int(box[0] * im_height),
            int(box[2] * im_height))

def mph_to_mps(mph):
    """
    Convert miles per hour to meters per second (floats)
    """
    return mph * 0.44704

def mps_to_mph(mps):
    """
    Convert meters per second to miles per hour (floats)
    """
    return mps / 0.44704

class Timing:
    """
    An object that can be used to time events with simple start, end and print
    functions.
    It maintains 3 internal dictionary to achieve this.
    """

    def __init__(self):
        self.start_times = {}
        self.durations = defaultdict(int)
        self.counts = defaultdict(int)

    def update_start(self, key):
        self.start_times[key] = time.time()

    def update_end(self, key, n=1):
        self.counts[key] += n
        self.durations[key] += time.time() - self.start_times[key]

    def print_stats(self):
        for key in self.start_times.keys():
            if self.counts[key] == 0: continue
            print("{} {} took: {}".format(
                key, self.counts[key], self.durations[key])
            )
