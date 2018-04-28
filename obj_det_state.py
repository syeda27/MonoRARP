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

'''
camera height in meters
camera_min_angle in degrees
rel_horizon is relative position of horizon in image. 0 <= x <= 1
'''
def bottom_bounding_box_distance(box, im_h, im_w, 
        rel_horizon=0.39, camera_min_angle=25.0, camera_height=1.0):
    horizon_p = rel_horizon * im_h
    (left, right, top, bot) = box
    d_image = im_h - bot # distance from bottom of image
    if d_image > horizon_p:
        print("this box is floating. Ignoring. Check horizon")
        return
    phi = ((horizon_p - d_image) / horizon_p) * (90.0 - camera_min_angle)
    return camera_height * np.tan(np.deg2rad(90.0 - phi))


# called after converting box
# box, im_h, im_w are pixels
# car width in meters
def state_estimation(box, im_h, im_w, camera_focal_length=1000, 
        car_width=2):
    state = dict()
    state["distance_triangle"] = triangle_similarity_distance(box, 
            camera_focal_length, car_width)
    d_bbb = bottom_bounding_box_distance(box, im_h, im_w)
    if d_bbb is not None:
        state["distance_bbb"] = d_bbb
    state["distance"] = np.mean([state[i] for i in state.keys() if "distance" in i])
    return state

# 3/4" at 8 inches away, Focal of about 1000 for built in webcam
def calibrate(box, im_h, im_w, object_width=0.019, distance=0.2032):
    (left, right, top, bot) = box
    object_width_pixels = right - left
    print(object_width_pixels)
    print("F= " + str((distance * object_width_pixels) / object_width))

def calibrate2(box, im_h, im_w, height=1.0):
    height = 1.0
    min_d = 2.5
    alpha = np.rad2deg(np.arctan(min_d))
    print("Alpha for min_d: ", str(alpha))
    print("Min observable distance: ", str(min_d))
