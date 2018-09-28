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
    (ymin, xmin, ymax, xmax) = b
    (left, right, top, bot) = (int(xmin * im_width), int(xmax * im_width),
                              int(ymin * im_height), int(ymax * im_height))
    return (left, right, top, bot)

