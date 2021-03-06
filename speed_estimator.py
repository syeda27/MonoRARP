"""
In order to estimate the abosulte ego speed, we define a class that is
designed to work independently of everything else, and just return the most
recent calculated speed when queried.

Author: Derek
"""

from driver_risk_utils import gps_utils, general_utils
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
        self.use_gps = launcher_args.use_gps
        if self.use_gps:
            self.gps_interface = gps_utils.GPS_Interface(launcher_args.gps_source)
        self.use_lane_markings = launcher_args.lane_based_speed
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

    def get_reading(self):
        """
        Returns average speed among active estimation methods in meters / second
        """
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
        if len(speed) == 0:
            return self.default_speed
        if self.verbose:
            print("Speed:", speed)
            print(sum(speed) / len(speed))
        return sum(speed) / len(speed)
