import numpy as np
from collections import defaultdict


'''
Use the state to keep track of states associated with vehicles over time. 

Simple use case is to store list of states, where bounding box index is 
the key to the stored state information.
Then, we reset the stored information whever the tracker is reset.

A more complex case is to store past state information and associate it with
the closest bounding box at query time.
We store a maximum amount of information at any given time.
'''
class state:
    states = defaultdict(list)
    MAX_HISTORY = 100

    '''
    states is the main component here.

    It is a dictionary that stores object key: past_state_info
    past_state_info is a list of dictionaries for this object.
    '''

    def __init__(self):
        self.states = defaultdict(list)

    # some helpers
    def clear(self):
        self.states = defaultdict(list)
    def get_state(self, object_key=None):
        if object_key is not None:
            return self.states[object_key]
        return self.states

    # TODO if no object key, finds closest box from last frame
    # F is camera focal length
    # W is car width in meters
    def update_state(self, box, im_h, im_w, F, W, object_key=1):
        if len(self.states[object_key]) >= self.MAX_HISTORY:
            self.states[object_key] = self.states[object_key][-self.MAX_HISTORY:]
        new_state = state_estimation(box, im_h, im_w, F, W)
        self.states[object_key].append(new_state)
        return new_state


# F is camera focal length
# W is car width in meters
# returns distance in meters
def triangle_similarity_distance(box, F, W):
    (left, right, top, bot) = box
    object_width_pixels = right - left
    return (W * F) / object_width_pixels

# called after converting box
# box, im_h, im_w are pixels
# car width in meters
def state_estimation(box, im_h, im_w, camera_focal_length=1000, 
        car_width=2):
    distance = triangle_similarity_distance(box, 
            camera_focal_length, car_width)
    return {"distance": distance}

# 3/4" at 8 inches away, Focal of about 1000 for built in webcam
def calibrate(box, im_h, im_w, object_width=0.019, distance=0.2032):
    (left, right, top, bot) = box
    object_width_pixels = right - left
    print(object_width_pixels)
    print("F= " + str((distance * object_width_pixels) / object_width))


