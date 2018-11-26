"""
The purpose of this file is to create a usable class out of the lane detection
scripts that Juan Carlos wrote, and integrate it with the DRIVR system.

I) IMAGE READING
II) LSD ALGORITHM
III) SCANNING OF THE IMAGE AND DETECTION OF ROAD MARK SIGNATURE
IV) ELIMINATION OF WHITE ROAD MARK DUPLICATES
V) MERGING ALL WHITE ROAD MARKS PREVIOUSLY DETECTED AND GENERATION OF TWO LANES
VI) FILTERING WHITE ROAD MARKS TO REMOVE FALSE DETECTIONS AND CORRECTION OF THE TWO LANES
VII) RECORDING OF CURRENTLY DETECTED LANES
VIII) GENERATION OF LONG TERM AVERAGE OF THE DETECTED LANES
IX) DISPLAY

Author: Juan Carlos Aragon  -  Allstate
Minor Editing: djp42  -  Stanford

See lane_detection_utils/README for more details.


"""

import cv2
import math
import numpy as np

from lane_detection_utils import lane_args_utils, absolute_speed, \
    average_delta_across_marking, \
    determination_of_parallelism_with_previously_tracked_lanes, \
    eliminate_duplicate_road_marks, filtering, lane_signature_detection, \
    long_term_average, merging_all_road_marks, \
    outlayer_removal_and_average_brightness, road_sampling, scanning_region
from driver_risk_utils import display_utils

class LaneDetector:
    def __init__(self,
                 scan_x_params=(1480, 2720, 80),
                 scan_y_params=(100, 250, 20),
                 scan_window_sz=(120, 160),
                 subframe_dims=(1500, 1800, 0, 3840),
                 display_lane_lines=False,
                 brightness_ratio_threshold=1.5,
                 horizontal_tolerance=50,
                 left_margin_detection=1500,
                 right_margin_detection=2700,
                 average_window=6):
        self.subframe_dims = subframe_dims
        lane_args_utils.initialize_lane_detector_members(self)
        #inital and end points for the scanning in the x-direction within the image subframe, and step of the scanning
        self.scan_x_ini, self.scan_x_end, self.scan_x_step = scan_x_params
        #inital and end points for the scanning in the y-direction within the image subframe, and step of the scanning
        self.scan_y_ini, self.scan_y_end, self.scan_y_step = scan_y_params
        #size of the rectangular window used for the scanning
        self.scanning_window_width, self.scanning_window_length = scan_window_sz
        self.image_number = 0
        self.display_lane_lines = display_lane_lines # If false, handle_image returns the lines
        self.brightness_ratio_threshold = brightness_ratio_threshold
        self.horizontal_tolerance = horizontal_tolerance
        self.left_margin_detection = left_margin_detection
        self.right_margin_detection = right_margin_detection
        self.average_window = average_window

    def _reset_each_image_vars(self):
        self.count_lanes = 0
        self.mux_lane_vec = np.zeros(40)
        self.muy_lane_vec = np.zeros(40)
        self.base_ptx_lane_vec = np.zeros(40)
        self.base_pty_lane_vec = np.zeros(40)
        self.angle_lanes = np.zeros(40)

    def handle_image(self, image):
        self.img = image
        self.img_subframe = self.img[
            self.subframe_dims[0]:self.subframe_dims[1],
            self.subframe_dims[2]:self.subframe_dims[3],
        ] # [1500:1800, 0:3849]
        self.img_subframe_gray = cv2.cvtColor(
            self.img_subframe, cv2.COLOR_BGR2GRAY)
        self.H, self.W, _ = self.img.shape
        self.h1, self.w1 = self.img_subframe_gray.shape
        self._reset_each_image_vars()
        self._do_line_segments(self.img_subframe_gray)
        self._scan_image_wrapper()
        self._everything_else()
        self.image_number += 1

    def _do_line_segments(self, img_subframe_gray):
        self.dlines = cv2.createLineSegmentDetector(0).detect(img_subframe_gray)
        #dlines holds the lines that have been detected by LSD
        for dline in self.dlines[0]:
            display_utils.make_line(img_subframe_gray,
                (round(dline[0][0]), round(dline[0][1])),
                (round(dline[0][2]), round(dline[0][3])))
            #we are redrawing the lines to emphasize their borders

    def _scan_image_wrapper(self):
        for xregion in range(self.scan_x_ini, self.scan_x_end, self.scan_x_step):
            for yregion in range(self.scan_y_ini, self.scan_y_end, self.scan_y_step):
                scan_args = scanning_region.scan_region(
                    self.dlines,
                    xregion,
                    yregion,
                    self.scanning_window_length,
                    self.scanning_window_width)
                self._scan_region_in_image(scan_args)

    def _scan_region_in_image(self, scan_args):
        # a) Scan region and generate line segments
        if scan_args.count_angles_per_region >= 2:
            #perform all the following processing only if 2 or more line-segments exists per scanning region
            # b) From all the lines we found inside a scaning region we find the two top line-segments (closest to the top of the frame).
            first_top = -1
            second_top = -1
            top = 10000
            for k1 in range(0, scan_args.count_angles_per_region):
                 if scan_args.ry1[k1] < top:
                     top = scan_args.ry1[k1]
                     first_top = k1
            top = 10000
            for k1 in range(0, scan_args.count_angles_per_region):
                 if scan_args.ry1[k1] < top and k1 != first_top:
                     top = scan_args.ry1[k1]
                     second_top = k1
            # There will be one top segment on the left and another on the right. Out of the two top segments indetify which segment is to the left and which to the right
            if scan_args.rx1[first_top] < scan_args.rx1[second_top]:
                top_left = first_top
                top_right = second_top
            else:
                top_left = second_top
                top_right = first_top
            # c) Sampling of 5 pixels around the two top line segments for each horizontal line crossing the two line segments, and detection of signature

            # criteria that must be complied with to proceed to the sampling:
            # 1) the top-left line and the top-right line should be aliged vertically so that horizontal lines would intersect both of them (at least for some portion of both)
            # 2) The difference between the top-left line's angle and the top-right line's angle should be less than 6 degrees
            if scan_args.ry2[top_left] > scan_args.ry1[top_right] \
                    and scan_args.ry2[top_right] > scan_args.ry1[top_left] \
                    and abs(scan_args.angles[top_right] - scan_args.angles[top_left]) < 6:
                # c1) Brightness Sampling in groups of 5 pixels (sample types: 4 samples for pavement and 1 sample for white road mark). One vector per sample type holding samples across vertical dim.
                road_sampling.brightness_sampling_w(self, scan_args, top_left, top_right)
                # c2) Artifact Removal and Computation of Average Brightness across the vertical dimension for each sample type
                outlayer_removal_and_average_brightness.o_r_a_a_b_c_wrap(self)

                # c3) Computation of the Average Delta that exist between each sample and the corresponding average computed across the vertical dimension (this is a meassure of dispersion)
                average_delta_across_marking.average_delta_w(self)

                # c4) Verification of alignment of the left top line segment with the lane tracked in the previous frame (important information to be used later)
                determination_of_parallelism_with_previously_tracked_lanes.determination_of_parallelism_w(
                    self, scan_args, top_left)
                # c5) Road Mark Signature Detetion for the two-top lines segments previously extracted from the scanning region
                lane_signature_detection.lane_signature_detection_w(self, scan_args, top_left, top_right)

                # c6) If Signature Detection fails but the two-top line segments are aligned with a previusly tracked lane we accept the two-top line segments as a white road mark
                if self.lane_signature_detected == 0 and \
                        self.aligned_to_tracked_lane == 1:
                    L_lane = (
                        (scan_args.rx1[top_left] - scan_args.rx2[top_left])**2 +
                        (scan_args.ry1[top_left] - scan_args.ry2[top_left])**2
                    )**0.5
                    mux_lane = (scan_args.rx1[top_left] - scan_args.rx2[top_left]) / L_lane
                    muy_lane = (scan_args.ry1[top_left] - scan_args.ry2[top_left]) / L_lane
                    #intersecting with top of image
                    Lintersection = -scan_args.ry1[top_left] / muy_lane
                    x1_lane = scan_args.rx1[top_left] + Lintersection * mux_lane
                    #intersection with bottom of image
                    Lintersection = (self.H - scan_args.ry1[top_left]) / muy_lane
                    x2_lane = scan_args.rx1[top_left] + Lintersection * mux_lane

                    # To display:
                    self.left_lane_points = [
                        (scan_args.rx1[top_left], scan_args.ry1[top_left]),
                        (scan_args.rx2[top_left], scan_args.ry2[top_left]),
                    ]
                    self.right_lane_points = [
                        (scan_args.rx1[top_right], scan_args.ry1[top_right]),
                        (scan_args.rx2[top_right], scan_args.ry2[top_right]),
                    ]

                    if self.display_lane_lines:
                        self.draw_lane_lines(self.img_subframe)

                    self.mux_lane_vec[self.count_lanes] = mux_lane
                    self.muy_lane_vec[self.count_lanes] = muy_lane
                    self.base_ptx_lane_vec[self.count_lanes] = scan_args.rx1[top_left]
                    self.base_pty_lane_vec[self.count_lanes] = scan_args.ry1[top_left]
                    self.count_lanes += 1

    def draw_lane_lines(self, image_to_draw_on, verbose=True):
        try:
            display_utils.make_line(image_to_draw_on,
                self.left_lane_points[0],
                self.left_lane_points[1],
                (0, 0, 255))
            display_utils.make_line(image_to_draw_on,
                self.right_lane_points[0],
                self.right_lane_points[1],
                (0, 255, 0))
        except AttributeError:
            if verbose:
                print("No lanes detected.")
            return

    def _everything_else(self):
        ######## ELIMINATION OF WHITE ROAD MARK DUPLICATES ########
        eliminate_duplicate_road_marks.eliminate_duplicate_road_marks_w(self)

        ######## MERGING ALL WHITE ROAD MARKS PREVIOUSLY DETECTED AND GENERATION OF TWO LANES ########
        merging_all_road_marks.merging_all_road_marks_w(self)

        ######## FILTERING WHITE ROAD MARKS TO REMOVE FALSE DETECTIONS AND CORRECTION OF THE TWO LANES #########
        if self.count_lanes > 0:
            filtering.filtering_w(self)

            ######## ABSOLUTE SPEED DETERMINATION ########
            absolute_speed.abs_speed_wrapper(self)

            ######## RECORDING OF CURRENTLY DETECTED LANES ########
            self.mux_lane_vec_final2_previous = self.mux_lane_vec_final2
            self.muy_lane_vec_final2_previous = self.muy_lane_vec_final2
            self.base_ptx_lane_vec_final2_previous = self.base_ptx_lane_vec_final2
            self.base_pty_lane_vec_final2_previous = self.base_pty_lane_vec_final2

            self.mux_lane_vec_final1_previous = self.mux_lane_vec_final1
            self.muy_lane_vec_final1_previous = self.muy_lane_vec_final1
            self.base_ptx_lane_vec_final1_previous = self.base_ptx_lane_vec_final1
            self.base_pty_lane_vec_final1_previous = self.base_pty_lane_vec_final1

            ######## GENERATION OF LONG TERM AVERAGE OF THE DETECTED LANES ########
            long_term_average.long_term_average_of_lanes_w(self)

        self.initial_frame_was_processed_flag = 1

    def display(self):
        """
        This is a bad way of doing things, so basically unused except in testing.
        """
        #resizing image for displaying purposes
        img7 = self.img_subframe
        dim = (3840, 2880)
        resized = cv2.resize(img7, dim, interpolation = cv2.INTER_CUBIC)

        if self.first_reading_available_flag != 0:
            speed_text = 'Speed: '+str(int(self.speed_official))+' miles/hr'
            cv2.putText(resized,
                        speed_text,
                        (250, 150),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        4,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA)

        cv2.namedWindow('Frame4', cv2.WINDOW_NORMAL)
        cv2.imshow('Frame4', resized )
        cv2.waitKey(1)
