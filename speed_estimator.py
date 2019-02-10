"""
In order to estimate the abosulte ego speed, we define a class that is
designed to work independently of everything else, and just return the most
recent calculated speed when queried.

Author: Derek
"""

from driver_risk_utils import gps_utils, general_utils, offline_utils
import threading
import lane_marking_speed_estimator

import queue
import threading

class SpeedEstimator():
    def __init__(
            self,
            launcher_args,
            default_speed=31.4,
            verbose=False,
            display_speed_lane=False):
        """
        Speed in meters per second
        """
        self.offline = launcher_args.offline
        self.save_path = launcher_args.results_save_path
        self.overwrite_saves = launcher_args.overwrite_saves
        self.component_name = "EGO_SPEED"
        self.load_inputs = launcher_args.L_EGO_SPEED
        self.path_to_load_inputs = launcher_args.prior_results_path
        self.use_gps = launcher_args.use_gps and not self.load_inputs
        if self.use_gps:
            self.gps_interface = gps_utils.GPS_Interface(launcher_args.gps_source)
        self.use_lane_markings = launcher_args.lane_based_speed and not self.load_inputs
        if self.use_lane_markings:
            self.lane_based_speed_interface = \
                lane_marking_speed_estimator.LaneMarkingSpeedEstimator(display_speed_lane)
        self.default_speed = default_speed
        self.verbose = verbose
        self.timer = general_utils.Timing()
        self.timer.update_start("Overall")

    def __del__(self):
        string = "\n============== Ending Speed Estimator =============="
        self.timer.update_end("Overall")
        string += "\nTiming:" + self.timer.print_stats(True)
        string += "\n=============="
        print(string)

    def update_estimates(self, image, frame_time):
        """
        This is unnecessary for the gps estimator, but necessary for most others
        """
        if self.use_lane_markings:
            self.timer.update_start("Lane Based Speed Update")
            self.lane_based_speed_interface.handle_image(image, frame_time)
            self.timer.update_end("Lane Based Speed Update")

    def load_speed(self, img_id):
        """
        Just a wrapper to help modularize get_reading()
        """
        speed = offline_utils.load_input(
            self.component_name,
            img_id,
            self.path_to_load_inputs,
            verbose=self.verbose
        )
        if self.verbose:
            print("{}: Successfully loaded tracked boxes of: {} from {} for img {}".format(
                self.component_name,
                net_out,
                self.path_to_load_inputs,
                img_id
            ))
        return speed

    def get_reading(self, img_id=None):
        """
        Returns average speed among active estimation methods in meters / second
        img_id:
          Integer. Used to specify how to save the data, if running in offline mode.
        """
        if self.load_inputs:
            return self.load_speed(img_id)
        speed = []
        if self.use_gps:
            self.timer.update_start("Get GPS Speed")
            speed.append(self.gps_interface.get_reading())
            print("gps_speed: ", str(speed[-1]))
            self.timer.update_end("Get GPS Speed")
        if self.use_lane_markings:
            self.timer.update_start("Get LBS Speed")
            speed.append(self.lane_based_speed_interface.get_speed())
            print("lane_speed: ", str(speed[-1]))
            self.timer.update_end("Get LBS Speed")
        if len(speed) > 0:
            avg_speed_all_methods = sum(speed) / len(speed)
        else:
            avg_speed_all_methods = self.default_speed
        if self.verbose:
            print("Speed:", speed)
            print(avg_speed_all_methods)
        if self.offline:
            offline_utils.save_output(avg_speed_all_methods, self.component_name,
                img_id, self.save_path, overwrite=self.overwrite_saves,
                verbose=self.verbose)
        return avg_speed_all_methods
