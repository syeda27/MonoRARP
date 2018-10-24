'''
This file contains some general utility functions used in the driver risk
estimation system.
'''

import time

def get_fps(start, frames):
    if frames < 5:
        return 1
    elapsed_time = time.time() - start
    return frames / elapsed_time

def check_aspect_ratio(box):
    # TODO add parameters
    (left, right, top, bot) = box
    width = right - left
    height = bot - top
    return width > 5 * height or height > 3 * width

# ymin, xmin, ymax, xmax  ===> left, right, top, bot
def convert(im_height, im_width, b):
    """
    Converts the box from normalized coordinates to absolute pixel values.
    """
    (ymin, xmin, ymax, xmax) = b
    (left, right, top, bot) = (int(xmin * im_width), int(xmax * im_width),
                              int(ymin * im_height), int(ymax * im_height))
    return (left, right, top, bot)

def mph_to_mps(mph):
    return mph * 0.44704
def mps_to_mph(mps):
    return mps / 0.44704

class timing:
    def __init__(self, keys):
        self.start_times = {}
        self.durations = {}
        self.counts = {}

        for key in keys:
            self.start_times[key] = time.time()
            self.durations[key] = 0
            self.counts[key] = 0

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
