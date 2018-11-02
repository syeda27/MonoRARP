"""
The purpose of this file is to create a usable class out of the lane detection
scripts that Juan Carlos wrote, and integrate it with the DRIVR system.

"""

import cv2
import math
import numpy as np

from lane_detection_utils import *
from driver_risk_utils import display_utils

class LaneDetctor:
    def __init__(self,
                 scan_x_params=(1480, 2720, 80),
                 scan_y_params=(100, 250, 20),
                 scan_window_sz=(120, 160)):
        lane_arg_utils.initialize_lane_detector_members(self)
        #inital and end points for the scanning in the x-direction within the image subframe, and step of the scanning
        self.scan_x_ini, self.scan_x_end, self.scan_x_step = scan_x_params
        #inital and end points for the scanning in the y-direction within the image subframe, and step of the scanning
        self.scan_y_ini, self.scan_y_end, self.scan_y_step = scan_y_params
        #size of the rectangular window used for the scanning
        self.scanning_window_width, self.scanning_window_length = scan_window_sz

    def reset_each_image_vars():
        self.count_lanes = 0
        self.mux_lane_vec = np.zeros(40)
        self.muy_lane_vec = np.zeros(40)
        self.base_ptx_lane_vec = np.zeros(40)
        self.base_pty_lane_vec = np.zeros(40)
        self.angle_lanes = np.zeros(40)

    def handle_image(self, image):
        img_subframe = image[1500:1800, 0:3849]
        img_subframe_gray = cv2.cvtColor(
            image, cv2.COLOR_BGR2GRAY)[1500:1800, 0:3849]
        H, W, _ = image.shape
        h1, w1 = image_subframe.shape
        self.reset_each_image_vars()
        dlines = self.do_line_segments(img_subframe_gray, display=True)



    def do_line_segments(self, image_subframe_gray, display=True):
        dlines = cv2.createLineSegmentDetector(0).detect(image_subframe_gray)
        #dlines holds the lines that have been detected by LSD
        if not display: return dlines
        for dline in dlines[0]:
            x0 = int(round(dline[0][0]))
            y0 = int(round(dline[0][1]))
            x1 = int(round(dline[0][2]))
            y1 = int(round(dline[0][3]))
            display_utils.make_line(image_subframe_gray,
                (round(dline[0][0]), round(dline[0][1])),
                (round(dline[0][2]), round(dline[0][3])))
            #we are redrawing the lines to emphasize their borders
        return dlines

    def scan_image_wrapper(self):
        for xregion in range(scan_x_ini, scan_x_end, scan_x_step):
            for yregion in range(scan_y_ini, scan_y_end, scan_y_step):
                scan_args = scan_region(
                    dlines,
                    xregion,
                    yregion,
                    scanning_window_length,
                    scanning_window_width)
                self.scan_region_in_image(scan_args)

    def scan_region_in_image(self, scan_args):
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
                (road1_vec, road2_vec, road3_vec, road4_vec,
                 whitemarkings_vec, counter_scanning) = brightness_sampling(
                    scan_args.rx1,
                    scan_args.rx2,
                    scan_args.ry1,
                    scan_args.ry2,
                    top_left,
                    top_right,
                    img_subframe_gray)
                # c2) Artifact Removal and Computation of Average Brightness across the vertical dimension for each sample type
                (road1_average, road2_average, road3_average, road4_average,
                whitemarkings_average, removed3,
                removed4) = outlayer_removal_and_average_brightness_computation(
                    road1_vec,
                    road2_vec,
                    road3_vec,
                    road4_vec,
                    whitemarkings_vec,
                    counter_scanning)

                # c3) Computation of the Average Delta that exist between each sample and the corresponding average computed across the vertical dimension (this is a meassure of dispersion)
                (delta_road1_average, delta_road2_average,
                delta_road3_average, delta_road4_average,
                delta_whitemarkings_average) = average_delta(
                    road1_vec,
                    road2_vec,
                    road3_vec,
                    road4_vec,
                    whitemarkings_vec,
                    road1_average,
                    road2_average,
                    road3_average,
                    road4_average,
                    whitemarkings_average,
                    removed3,
                    removed4,
                    counter_scanning)
                # c4) Verification of alignment of the left top line segment with the lane tracked in the previous frame (important information to be used later)
                aligned_to_tracked_lane = determination_of_parallelism(
                    top_left,
                    scan_args.rx1,
                    scan_args.ry1,
                    count_lane_group1,
                    count_lane_group2,
                    whitemarkings_average,
                    road1_average,
                    base_ptx_lane_vec_final1,
                    base_pty_lane_vec_final1,
                    mux_lane_vec_final1,
                    muy_lane_vec_final1,
                    base_ptx_lane_vec_final2,
                    base_pty_lane_vec_final2,
                    mux_lane_vec_final2,
                    muy_lane_vec_final2,
                    scan_args.angles)
                # c5) Road Mark Signature Detetion for the two-top lines segments previously extracted from the scanning region
                (lane_signature_detected, mux_lane_vec, muy_lane_vec,
                base_ptx_lane_vec, base_pty_lane_vec,
                count_lanes) = lane_signature_detection(
                    road1_average,
                    road2_average,
                    road3_average,
                    road4_average,
                    delta_road1_average,
                    delta_road2_average,
                    delta_road3_average,
                    delta_road4_average,
                    whitemarkings_average,
                    delta_whitemarkings_average,
                    scan_args.rx1,
                    scan_args.rx2,
                    scan_args.ry1,
                    scan_args.ry2,
                    top_left,
                    top_right,
                    mux_lane_vec,
                    muy_lane_vec,
                    base_ptx_lane_vec,
                    base_pty_lane_vec,
                    count_lanes,
                    H,
                    img_subframe)
                # c6) If Signature Detection fails but the two-top line segments are aligned with a previusly tracked lane we accept the two-top line segments as a white road mark
                if lane_signature_detected == 0 and \
                        aligned_to_tracked_lane == 1:
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
                    Lintersection = (H - scan_args.ry1[top_left]) / muy_lane
                    x2_lane = scan_args.rx1[top_left] + Lintersection * mux_lane
                    cv2.line(img_subframe,
                             (int(scan_args.rx1[top_left]), int(scan_args.ry1[top_left])),
                             (int(scan_args.rx2[top_left]), int(scan_args.ry2[top_left])),
                             (0, 0, 255),
                             1,
                             cv2.LINE_AA)
                    cv2.line(img_subframe,
                             (int(scan_args.rx1[top_right]), int(scan_args.ry1[top_right])),
                             (int(scan_args.rx2[top_right]), int(scan_args.ry2[top_right])),
                             (0, 255, 0),
                             1,
                             cv2.LINE_AA)
                    mux_lane_vec[count_lanes] = mux_lane
                    muy_lane_vec[count_lanes] = muy_lane
                    base_ptx_lane_vec[count_lanes] = scan_args.rx1[top_left]
                    base_pty_lane_vec[count_lanes] = scan_args.ry1[top_left]
                    count_lanes += 1
