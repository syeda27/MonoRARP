import numpy as np
import sys
sys.stdout.flush()
from collections import defaultdict

from driver_risk_utils import general_utils
from driver_risk_utils import state_estimation_utils as s_utils

"""
Use the state to keep track of states associated with vehicles over time.

Simple use case is to store list of states, where bounding box index is
the key to the stored state information.
Then, we reset the stored information whever the tracker is reset.

A more complex case is to store past state information and associate it with
the closest bounding box at query time.
We store a maximum amount of information at any given time.
"""
class state:
    MAX_HISTORY = 100

    """
    states is the main component here.

    It is a dictionary that stores object key: past_state_info
    past_state_info is a list of dictionaries for this object.
    """

    def __init__(self):
        self.state_histories = defaultdict(list)
        # A dictionary of vehicle ID: list of vehicle state history
        self.ego_speed = 0

    # some helpers
    def clear(self):
        self.state_histories = defaultdict(list)

    def get_state(self, object_key=None):
        if object_key is not None:
            return self.state_histories[object_key]
        return self.state_histories

    def get_current_states(self, object_key=None):
        if object_key is not None:
            return self.state_histories[object_key][-1]
        current_state = {}
        for vehicle_id, state_history in self.state_histories.items():
            current_state[vehicle_id] = state_history[-1]
        return current_state

    def get_ego_speed(self):
        return self.ego_speed
    def get_ego_speed_mph(self):
        return general_utils.mps_to_mph(self.ego_speed)

    def set_ego_speed(self, speed_mps):
        self.ego_speed = speed_mps
    def set_ego_speed_mph(self, speed_mph):
        self.ego_speed = general_utils.mph_to_mps(speed_mph)

    def update_distance(self, args, box, im_h, im_w, object_key):
        state = dict()
        if s_utils.get_distance_far_box_edge(box, im_w) < im_w / 10:
            # when further off center than this, we do not trust this distance.
            state["distance_y_t"] = s_utils.triangle_similarity_distance(box, args.focal, args.carW)
        d_bbb = s_utils.bottom_bounding_box_distance(box, im_h, im_w,
                camera_height=args.cameraH,
                camera_min_angle=args.cameraMinAngle,
                camera_beta_max=args.cameraMaxHorizAngle,
                carW = args.carW,
                rel_horizon=args.horizon)
        if d_bbb is not None:
            state["distance_y_b"], state["distance_x_b"] = d_bbb
        state["distance_y_b2"], state["distance_x_b2"] = \
                s_utils.bottom_bounding_box_distance2(box, im_h, im_w,
                        camera_focal_len = args.focal,
                        camera_height = args.cameraH, carW=args.carW)
        state["distance_x"] = s_utils.left_of_center(box, im_w) * \
                np.mean([state[i] for i in state.keys() if "distance_x" in i])
        state["distance_y"] = np.mean([state[i] for i in state.keys() if "distance_y" in i])
        self.state_histories[object_key].append(state)

    def update_speed(self, object_key):
        S = s_utils.calc_speed(self.state_histories[object_key])
        Sy = None
        Sx = None
        if S is not None:
            Sy, Sx = S
        if Sy is not None:
            self.state_histories[object_key][-1]["speed_y"] = Sy
        if Sx is not None:
            self.state_histories[object_key][-1]["speed_x"] = Sx

    def log_test_output(self, args, box, im_h, im_w, object_key):
        print("==================================")
        print("Object:", object_key)
        print("TRIANGLE")
        distance_to_far_box_edge = s_utils.get_distance_far_box_edge(box, im_w)
        print("is centered?", distance_to_far_box_edge, im_w / 10)
        print(distance_to_far_box_edge < im_w / 10)
        print("dy:", s_utils.triangle_similarity_distance(box, args.focal, args.carW))
        print("Bounding Box 1")
        s_utils.bottom_bounding_box_distance(box, im_h, im_w,
            camera_height=args.cameraH,
            camera_min_angle=args.cameraMinAngle,
            camera_beta_max=args.cameraMaxHorizAngle,
            carW = args.carW,
            rel_horizon=args.horizon, verbose=True)
        print("Bounding Box 2")
        s_utils.s_utils.bottom_bounding_box_distance2(box, im_h, im_w, args.focal,
                args.cameraH, carW=args.carW, verbose=True)
        print("Average distance y:", self.state_histories[object_key][-1]["distance_y"])
        print("Average distance x:", self.state_histories[object_key][-1]["distance_x"])
        s_utils.calc_speed(self.state_histories[object_key], verbose=True)
        print("==================================")


    # TODO if no object key, finds closest box from last frame
    # args needs camera (focal, height, minAngle), horizon, carW
    # called after converting box
    # box, im_h, im_w are pixels
    # car width in meters
    # TODO smooth distance
    def update_state(self, box, im_h, im_w, args, object_key=1,
            test=False, do_calibrate=False):
        state_len = len(self.state_histories[object_key])
        if state_len >= self.MAX_HISTORY:
            self.state_histories[object_key] = self.state_histories[object_key][-(self.MAX_HISTORY-1):]
        self.update_distance(args, box, im_h, im_w, object_key)
        self.update_speed(object_key)

        if do_calibrate:
            s_utils.calibrate(box, im_h, im_w)
        if test:
            self.log_test_output(args, box, im_h, im_w, object_key)
        return self.state_histories[object_key][-1]
