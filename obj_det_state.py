import numpy as np
import sys
sys.stdout.flush()
from collections import defaultdict

from driver_risk_utils import general_utils
from driver_risk_utils import state_estimation_utils as s_utils

class state:
    """
    Use the state to keep track of states associated with vehicles over time.
    The "states" are quantaties such as relative position and velocity.
    The state is simply a class that tracks some things like absolute ego speed
    and the history of "states"

    state_histories is the main component here.
    It is a dictionary that stores object key: past_state_info
    past_state_info is a list of dictionaries for this object.

    We store a maximum amount of information at any given time.

    ego_speed is in meters per second. We provide helpers to convert from mph.
    It represents the absolute speed for the ego vehicle.
    """
    MAX_HISTORY = 100

    def __init__(self):
        self.state_histories = defaultdict(list)
        # A dictionary of vehicle ID: list of vehicle state history
        self.ego_speed = 0

    # some helpers
    def clear(self):
        """
        Resets self.state_histories to an empty dictionary.
        """
        self.state_histories = defaultdict(list)

    def get_state(self, object_key=None):
        """
        Arguments:
          object_key: string
            An optional key (e.g. a vehicle ID).
            (A) If not None, return just the history for the given object.
            (B) If None, return all state_histories.

        Return:
          history: (A) list < dict<string : float> >
                or (B) dict < string:  list < dict<string : float> > >
            the state information history for a given object or all objects.
        """
        if object_key is not None:
            return self.state_histories[object_key]
        return self.state_histories

    def get_current_states(self, object_key=None):
        """
        Similar to get_state, but only the most current information.

        Arguments:
          object_key: string
            An optional key (e.g. a vehicle ID).
            (A) If not None, return just the state for the given object.
            (B) If None, return all current states.

        Return:
          history: (A) dict<string : float>
                or (B) dict < string:  dict<string : float> >
            the state information history for a given object or all objects.
        """
        if object_key is not None:
            return self.state_histories[object_key][-1]
        current_state = {}
        for vehicle_id, state_history in self.state_histories.items():
            current_state[vehicle_id] = state_history[-1]
        return current_state

    def get_ego_speed(self):
        """
        Returns the stored absolute ego vehicle speed, in meters/sec.
        """
        return self.ego_speed
    def get_ego_speed_mph(self):
        """
        Returns the stored absolute ego vehicle speed, in miles/hr.
        """
        return general_utils.mps_to_mph(self.ego_speed)

    def set_ego_speed(self, speed_mps):
        """
        Sets the stored absolute ego vehicle speed, given speed in meters/sec.
        """
        self.ego_speed = speed_mps
    def set_ego_speed_mph(self, speed_mph):
        """
        Sets the stored absolute ego vehicle speed, given speed in miles/hr.
        """
        self.ego_speed = general_utils.mph_to_mps(speed_mph)

    def log_test_output(self, args, box, im_h, im_w, object_key):
        """
        A function to log a lot of information about the state estimation
        process. Same function prototype as update_distance, with a lot of
        printing and logging performed to check calculations.
        """
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
    # TODO smooth distance
    def update_state(self,
                     box,
                     im_h,
                     im_w,
                     args,
                     object_key=1,
                     test=False,
                     do_calibrate=False):
        """
        For the given object key and associated detection box, we update our
        state information. This updates both distance and speed and should
        be treated as the external interface to the state class.

        TODO: pass in time image was captured.
        TODO: make state an object / struct that also includes time.

        Arguments:
          box: (int, int, int, int)
            (left, right, top, bottom) coordinates of a box in the image.
          im_h: int
            The height of the image, in pixels.
          im_w: int
            The width of the image, in pixels.
          args: An args object as from argument_utils.py. Must contain
            information like camera parameters, car width, etc.
          object_key: string
            The key that corresponds to this detected object.
          test: boolean
            Whether or not we want to log the test output to make sure the
            calculations being made are correct.
          do_calibrate: boolean
            Whether or not we want to run the calibrate function from the
            state estimation utilities.

        Returns:
          state: dict <string : float>
            The new state that we created, of entries like "distance_y": 1.0
            (It is also appended to the internal state_histories.)
        """

        state_len = len(self.state_histories[object_key])
        if state_len >= self.MAX_HISTORY:
            self.state_histories[object_key] = self.state_histories[object_key][-(self.MAX_HISTORY-1):]
        self._update_distance(args, box, im_h, im_w, object_key)
        self._update_speed(object_key)

        if do_calibrate:
            s_utils.calibrate(box, im_h, im_w)
        if test:
            self.log_test_output(args, box, im_h, im_w, object_key)
        return self.state_histories[object_key][-1]

    # Private
    def _update_distance(self, args, box, im_h, im_w, object_key):
        """
        For the given object key and associated detection box, we update our
        state information, but only the distances.
        This should only be called internally, hence the "_".

        TODO: pass in time image was captured.
        TODO: make state an object / struct that also includes time.

        Arguments:
          args: An args object as from argument_utils.py. Must contain
            information like camera parameters, car width, etc.
          box: (int, int, int, int)
            (left, right, top, bottom) coordinates of a box in the image.
          im_h: int
            The height of the image, in pixels.
          im_w: int
            The width of the image, in pixels.
          object_key: string
            The key that corresponds to this detected object.

        """
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

    def _update_speed(self, object_key):
        """
        Uses the stored state_histories to update the speed for the given
        object.
        This should only be called internally, hence the "_".

        Arguments:
          object_key: string
            The object that we wish to update the speed for.

        Note:
          Because state_histories is a default dict, if the object does not
          exist in the state_histories, we will get an S of None from utils,
          and thus never update the speed.
        """
        S = s_utils.calc_speed(self.state_histories[object_key])
        Sy = None
        Sx = None
        if S is not None:
            Sy, Sx = S
        if Sy is not None:
            self.state_histories[object_key][-1]["speed_y"] = Sy
        if Sx is not None:
            self.state_histories[object_key][-1]["speed_x"] = Sx
