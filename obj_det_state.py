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

    # TODO direction position/distance and velocity
    # TODO if no object key, finds closest box from last frame
    # args needs camera (focal, height, minAngle), horizon, carW
    # called after converting box
    # box, im_h, im_w are pixels
    # car width in meters
    # TODO smooth distance
    def update_state(self, box, im_h, im_w, args, object_key=1, test=True):
        state_len = len(self.states[object_key])
        if state_len >= self.MAX_HISTORY:
            self.states[object_key] = self.states[object_key][-(self.MAX_HISTORY-1):]
        state = dict()
        (left, right, top, bot) = box
        center_bottom_box = left + (left - right) / 2
        center_image = im_w / 2
        if abs(center_bottom_box - center_image) < im_w / 10:
            # when further off center than this, we do not trust this distance.
            state["distance_y_t"] = triangle_similarity_distance(box, args.focal, args.carW)
        d_bbb = bottom_bounding_box_distance(box, im_h, im_w,
                camera_height=args.cameraH, 
                camera_min_angle=args.cameraMinAngle, 
                rel_horizon=args.horizon)
        if d_bbb is not None:
            state["distance_y_b"], state["distance_x_b"] = d_bbb
            state["distance_x"] = np.mean([state[i] for i in state.keys() if "distance_x" in i])
        state["distance_y"] = np.mean([state[i] for i in state.keys() if "distance_y" in i])
        self.states[object_key].append(state)
        S = calc_speed(self.states[object_key], verbose=test)
        Sy = None
        Sx = None
        if S is not None:
            Sy, Sx = S
        if Sy is not None:
            self.states[object_key][-1]["speed_y"] = Sy
        if Sx is not None:
            self.states[object_key][-1]["speed_x"] = Sx
        if test:
            print("==================================")
            print("Object:", object_key)
            print("TRIANGLE")
            print("is centered?", abs(center_bottom_box - center_image), im_w / 10)
            print(abs(center_bottom_box - center_image) < im_w / 10)
            print("dy:", triangle_similarity_distance(box, args.focal, args.carW))
            print("Bounding Box 1")
            bottom_bounding_box_distance(box, im_h, im_w,
                camera_height=args.cameraH, 
                camera_min_angle=args.cameraMinAngle, 
                rel_horizon=args.horizon, verbose=True)
            print("Bounding Box 2")
            bottom_bounding_box_distance2(box, im_h, im_w, args.focal,
                    args.cameraH, verbose=True)
            print("Average distance y:", self.states[object_key][-1]["distance_y"])
            print("Average distance y:", self.states[object_key][-1]["distance_x"])
            print("==================================")
        return self.states[object_key][-1]

# TODO don't assume uniform frame rate - could record time
# right now this function returns the average distance change per frame 
#  from the last TO_USE  frames. 
# If there is no history, aka this is the first frame, it returns None so
#  that it will be ignored.
# state_for_object is the list of states (history) identified by an object.
# Also, average with the previous calculated speed, if it exists.
def calc_speed(state_for_object, TO_USE=5, verbose=False):
    if (len(state_for_object)) <= 1:
        return None
    to_consider = state_for_object[-TO_USE:]
    Dx = 0
    nx = 0
    Dy = 0
    ny = 0
    for i in range(len(to_consider) - 1):
        if "distance_y" in to_consider[i+1] and "distance_y" in to_consider[i]:

            Dy += (to_consider[i+1]['distance_y'] - to_consider[i]['distance_y'])
            ny += 1
        if "distance_x" in to_consider[i+1] and "distance_x" in to_consider[i]:

            Dx += (to_consider[i+1]['distance_x'] - to_consider[i]['distance_x'])
            nx += 1
    Sy = None
    Sx = None
    if ny > 0:
        Sy = Dy / ny
    if nx > 0:
        Sx = Dx / nx
    if verbose:
        print("dy:", Dy, "ny:", ny)
        print("dx:", Dx, "nx:", nx)
        print("Sy:",Sy, "Sx:",Sx)
    return (Sy, Sx)



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

return dy (along centerline) and dx (perpendicular to that, aka horizontal)
'''
def bottom_bounding_box_distance(box, im_h, im_w, 
        rel_horizon=0.39, camera_min_angle=25.0, camera_height=1.0,
        verbose=False):
    horizon_p = rel_horizon * im_h
    (left, right, top, bot) = box
    d_image = im_h - bot # distance from bottom of image
    if d_image > horizon_p:
        if verbose: print("this box is floating. ignoring. check horizon")
        return none
    phi = ((horizon_p - d_image) / horizon_p) * (90.0 - camera_min_angle)
    dy = camera_height / np.tan(np.deg2rad(phi))
    dx = triangle_for_x(box, im_w, d_image, dy)
    if verbose:
        print("dy:", dy, "dx:", dx) 
    return (dy, dx)

# to get dx, we assume the triangle similarity properties hold
def triangle_for_x(box, im_w, d_image, dy):
    (left, right, top, bot) = box
    center_bottom_box = left + ((left - right) / 2)
    center_image = im_w / 2
    # todo d_image = 0
    dx = dy * (abs(center_bottom_box-center_image)/(d_image+0.0001))
    return dx

def bottom_bounding_box_distance2(box, im_h, im_w, 
        camera_focal_len=1000, camera_height=1.0, verbose=False):
    (left, right, top, bot) = box
    d_image = im_h - bot # distance from bottom of image
    dy = (camera_height * camera_focal_len) / d_image
    dx = triangle_for_x(box, im_w, d_image, dy)
    if verbose:
        print("dy:", dy, "dx:", dx) 
    return (dy, dx)

# 3/4" at 8 inches away, Focal of about 1000 for built in webcam
def calibrate(box, im_h, im_w, object_width=0.019, distance=0.2032):
    (left, right, top, bot) = box
    object_width_pixels = right - left
    print(object_width_pixels)
    print("F= " + str((distance * object_width_pixels) / object_width))

def calibrate2(box, im_h, im_w, height=1.3462):
    min_d = 1.8288
    alpha = np.rad2deg(np.arctan(min_d/height))
    print("Alpha for min_d: ", str(alpha))
    print("Min observable distance: ", str(min_d))
