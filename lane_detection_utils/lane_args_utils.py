"""
There are so many variables related to the lane detector, that I think it makes
sense to create a set of classes for them...
"""

import numpy as np

class scan_region_output_args:
    def __init__(self):
        #Initializing vectors to be used for collection of the lines inside the region
        self.rx1 = np.zeros(100)
        self.rx2 = np.zeros(100)
        self.ry1 = np.zeros(100)
        self.ry2 = np.zeros(100)
        #Initializing vector for angle collection
        self.angles = np.zeros(100)
        self.count_angles_per_region = 0

    def print(self):
        print(
            self.rx1,
            self.rx2,
            self.ry1,
            self.angles,
            self.count_angles_per_region
        )


def initialize_lane_detector_members(lane_detector_object):
    lane_detector_object.mux_lane_vec_previous = np.zeros(40)
    lane_detector_object.muy_lane_vec_previous = np.zeros(40)
    lane_detector_object.base_ptx_lane_vec_previous = np.zeros(40)
    lane_detector_object.base_pty_lane_vec_previous = np.zeros(40)

    lane_detector_object.mux_lane_vec_aggregated = np.zeros(40)
    lane_detector_object.muy_lane_vec_aggregated = np.zeros(40)
    lane_detector_object.base_ptx_lane_vec_aggregated = np.zeros(40)
    lane_detector_object.base_pty_lane_vec_aggregated = np.zeros(40)

    lane_detector_object.count_lanes_average_vec = 0
    lane_detector_object.mux_lane_vec_average = np.zeros(4000)
    lane_detector_object.muy_lane_vec_average = np.zeros(4000)
    lane_detector_object.base_ptx_lane_vec_average = np.zeros(4000)
    lane_detector_object.base_pty_lane_vec_average = np.zeros(4000)

    lane_detector_object.count_lanes_average_vec2 = 0
    lane_detector_object.mux_lane_vec_average2 = np.zeros(4000)
    lane_detector_object.muy_lane_vec_average2 = np.zeros(4000)
    lane_detector_object.base_ptx_lane_vec_average2 = np.zeros(4000)
    lane_detector_object.base_pty_lane_vec_average2 = np.zeros(4000)

    lane_detector_object.count_lane_group1 = 0
    lane_detector_object.count_lane_group2 = 0

    lane_detector_object.white_mark_hit = 0
    lane_detector_object.count_road_nomark = 0
    lane_detector_object.capture_frameindex_for_speed = 0
    lane_detector_object.frameindex_for_speed_previous = 0
    lane_detector_object.frameindex_for_speed = 0
    lane_detector_object.count_scanned_lines_reverse_for_speed_previous = 0
    lane_detector_object.count_scanned_lines_reverse_for_speed = 0

    lane_detector_object.white_mark_hit_1 = 0
    lane_detector_object.count_road_nomark_1 = 0
    lane_detector_object.capture_frameindex_for_speed_1 = 0
    lane_detector_object.frameindex_for_speed_previous_1 = 0
    lane_detector_object.frameindex_for_speed_1 = 0
    lane_detector_object.count_scanned_lines_reverse_for_speed_previous_1 = 0
    lane_detector_object.count_scanned_lines_reverse_for_speed_1 = 0

    lane_detector_object.base_ptx_lane_vec_final1 = 0
    lane_detector_object.base_pty_lane_vec_final1 = 0
    lane_detector_object.mux_lane_vec_final1 = 0
    lane_detector_object.muy_lane_vec_final1 = 0

    lane_detector_object.base_ptx_lane_vec_final2 = 0
    lane_detector_object.base_pty_lane_vec_final2 = 0
    lane_detector_object.mux_lane_vec_final2 = 0
    lane_detector_object.muy_lane_vec_final2 = 0

    lane_detector_object.mux_lane_vec_final1_previous = 0
    lane_detector_object.muy_lane_vec_final1_previous = 0
    lane_detector_object.base_ptx_lane_vec_final1_previous = 0
    lane_detector_object.base_pty_lane_vec_final1_previous = 0

    lane_detector_object.mux_lane_vec_final2_previous = 0
    lane_detector_object.muy_lane_vec_final2_previous = 0
    lane_detector_object.base_ptx_lane_vec_final2_previous = 0
    lane_detector_object.base_pty_lane_vec_final2_previous = 0

    lane_detector_object.x1_lane_group1 = 0
    lane_detector_object.x1_lane_group2 = 0

    lane_detector_object.initial_frame_was_processed_flag = 0

    lane_detector_object.first_reading_available_flag = 0
