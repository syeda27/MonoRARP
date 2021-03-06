
import numpy as np
import sys

sys.path.append("..")

"""
LANE DETECTION MAIN

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

See README for more details.

"""

import cv2
import math
import numpy as np

sys.path.append("..")
sys.path.append("../lane_detection_utils")

from absolute_speed import absolute_speed_estimation
from scanning_region import scan_region
from road_sampling import brightness_sampling
from outlayer_removal_and_average_brightness import outlayer_removal_and_average_brightness_computation
from average_delta_across_marking import average_delta
from determination_of_parallelism_with_previously_tracked_lanes import determination_of_parallelism
from lane_signature_detection import lane_signature_detection
from eliminate_duplicate_road_marks import eliminate_duplicate_road_marks
from merging_all_road_marks import merging_all_road_marks
from filtering import filtering
from long_term_average import long_term_average_of_lanes
import lane_args_utils


##################### INITIALIZATION OF VARIABLES ###########################

mux_lane_vec_previous = np.zeros(40)
muy_lane_vec_previous = np.zeros(40)
base_ptx_lane_vec_previous = np.zeros(40)
base_pty_lane_vec_previous = np.zeros(40)

mux_lane_vec_aggregated = np.zeros(40)
muy_lane_vec_aggregated = np.zeros(40)
base_ptx_lane_vec_aggregated = np.zeros(40)
base_pty_lane_vec_aggregated = np.zeros(40)

count_lanes_average_vec = 0
mux_lane_vec_average = np.zeros(4000)
muy_lane_vec_average = np.zeros(4000)
base_ptx_lane_vec_average = np.zeros(4000)
base_pty_lane_vec_average = np.zeros(4000)

count_lanes_average_vec2 = 0
mux_lane_vec_average2 = np.zeros(4000)
muy_lane_vec_average2 = np.zeros(4000)
base_ptx_lane_vec_average2 = np.zeros(4000)
base_pty_lane_vec_average2 = np.zeros(4000)

count_lane_group1 = 0
count_lane_group2 = 0

white_mark_hit = 0
count_road_nomark = 0
capture_frameindex_for_speed = 0
frameindex_for_speed_previous = 0
frameindex_for_speed = 0
count_scanned_lines_reverse_for_speed_previous = 0
count_scanned_lines_reverse_for_speed = 0


white_mark_hit_1 = 0
count_road_nomark_1 = 0
capture_frameindex_for_speed_1 = 0
frameindex_for_speed_previous_1 = 0
frameindex_for_speed_1 = 0
count_scanned_lines_reverse_for_speed_previous_1 = 0
count_scanned_lines_reverse_for_speed_1 = 0

base_ptx_lane_vec_final1 = 0
base_pty_lane_vec_final1 = 0
mux_lane_vec_final1 = 0
muy_lane_vec_final1 = 0

base_ptx_lane_vec_final2 = 0
base_pty_lane_vec_final2 = 0
mux_lane_vec_final2 = 0
muy_lane_vec_final2 = 0

mux_lane_vec_final1_previous = 0
muy_lane_vec_final1_previous = 0
base_ptx_lane_vec_final1_previous = 0
base_pty_lane_vec_final1_previous = 0

mux_lane_vec_final2_previous = 0
muy_lane_vec_final2_previous = 0
base_ptx_lane_vec_final2_previous = 0
base_pty_lane_vec_final2_previous = 0

x1_lane_group1 = 0
x1_lane_group2 = 0


initial_frame_was_processed_flag = 0

first_reading_available_flag = 0


########### CONFIGURABLE PARAMETERS #################

#inital and end points for the scanning in the x-direction within the image subframe, and step of the scanning
scan_x_ini = 1480
scan_x_end = 2720
scan_x_step = 80

#inital and end points for the scanning in the y-direction within the image subframe, and step of the scanning
scan_y_ini = 100
scan_y_end = 250
scan_y_step = 20

#size of the rectangular window used for the scanning
scanning_window_width = 120
scanning_window_length = 160


########### BEGINNING OF LANE DETECTION ################

for image_number in range(1085, 2665):


    ######## IMAGE READING ########

    #Reading of Image Frame
    image_name = '../tests/Hwy101_frames/'+str(image_number)+'.jpg'
    image = cv2.imread(image_name)                     #image for visualization of results (in color)
    img_subframe = image[1500:1800, 0:3849]                     #image subframe for visualization of results (in color)

    img_subframe_gray = cv2.imread(image_name, cv2.IMREAD_GRAYSCALE)[1500:1800, 0:3849]	 #image for processing algorithm
    #img_subframe_gray = img[1500:1800, 0:3849]			      #image subframe that is closer to the bottom part of the frame where markings are more visible.

    H, W, _  = image.shape
    h1, w1 =  img_subframe_gray.shape

    count_lanes = 0

    #Initialization on every frame
    mux_lane_vec = np.zeros(40)
    muy_lane_vec = np.zeros(40)
    base_ptx_lane_vec = np.zeros(40)
    base_pty_lane_vec = np.zeros(40)
    angle_lanes = np.zeros(40)




    ######## LSD ALGORITHM ########

    #LSD algorithm object creation
    lsd = cv2.createLineSegmentDetector(0)

    #LSD line segment detection
    dlines = lsd.detect(img_subframe_gray)  #dlines holds the lines that have been detected by LSD
    for dline in dlines[0]:
        x0 = int(round(dline[0][0]))
        y0 = int(round(dline[0][1]))
        x1 = int(round(dline[0][2]))
        y1 = int(round(dline[0][3]))
        cv2.line(img_subframe_gray, (x0, y0), (x1, y1), 255, 1, cv2.LINE_AA)  #we are redrawing the lines to emphasize their borders


    ######## SCANNING OF THE IMAGE AND DETECTION OF ROAD MARK SIGNATURE ########

    for xregion in range(scan_x_ini, scan_x_end, scan_x_step):
        for yregion in range(scan_y_ini, scan_y_end, scan_y_step):
            # a) Scan region and generate line segments
            scan_args = scan_region(
                dlines,
                xregion,
                yregion,
                scanning_window_length,
                scanning_window_width)
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

    ######## ELIMINATION OF WHITE ROAD MARK DUPLICATES ########
    (mux_lane_vec_aggregated, muy_lane_vec_aggregated,
     base_ptx_lane_vec_aggregated, base_pty_lane_vec_aggregated,
     angle_lanes, count_tracked_lanes2) = \
        eliminate_duplicate_road_marks(
            mux_lane_vec,
            muy_lane_vec,
            base_ptx_lane_vec,
            base_pty_lane_vec,
            count_lanes,
            mux_lane_vec_aggregated,
            muy_lane_vec_aggregated,
            base_ptx_lane_vec_aggregated,
            base_pty_lane_vec_aggregated,
            angle_lanes)

    ######## MERGING ALL WHITE ROAD MARKS PREVIOUSLY DETECTED AND GENERATION OF TWO LANES ########
    (mux_lane_vec_final1, muy_lane_vec_final1, base_ptx_lane_vec_final1,
     base_pty_lane_vec_final1, mux_lane_vec_final2, muy_lane_vec_final2,
     base_ptx_lane_vec_final2, base_pty_lane_vec_final2,
     count_lane_group1, count_lane_group2) = merging_all_road_marks(
        angle_lanes,
        count_tracked_lanes2,
        mux_lane_vec_aggregated,
        muy_lane_vec_aggregated,
        base_ptx_lane_vec_aggregated,
        base_pty_lane_vec_aggregated)

    ######## FILTERING WHITE ROAD MARKS TO REMOVE FALSE DETECTIONS AND CORRECTION OF THE TWO LANES #########
    (mux_lane_vec_final1, muy_lane_vec_final1, base_ptx_lane_vec_final1,
     base_pty_lane_vec_final1, mux_lane_vec_final2, muy_lane_vec_final2,
     base_ptx_lane_vec_final2, base_pty_lane_vec_final2,
     x1_lane_group1, x1_lane_group2) = filtering(
        count_lane_group1,
        count_lane_group2,
        x1_lane_group1,
        x1_lane_group2,
        H,
        initial_frame_was_processed_flag,
        mux_lane_vec_final1,
        muy_lane_vec_final1,
        base_ptx_lane_vec_final1,
        base_pty_lane_vec_final1,
        mux_lane_vec_final1_previous,
        muy_lane_vec_final1_previous,
        base_ptx_lane_vec_final1_previous,
        base_pty_lane_vec_final1_previous,
        mux_lane_vec_final2,
        muy_lane_vec_final2,
        base_ptx_lane_vec_final2,
        base_pty_lane_vec_final2,
        mux_lane_vec_final2_previous,
        muy_lane_vec_final2_previous,
        base_ptx_lane_vec_final2_previous,
        base_pty_lane_vec_final2_previous)

    ######## ABSOLUTE SPEED DETERMINATION ########
    #Speed on left track (group2)
    if count_lane_group2 >= 1:
         muy_lane_vec_final2_speed = muy_lane_vec_final2
         mux_lane_vec_final2_speed = mux_lane_vec_final2
         base_ptx_lane_vec_final2_speed = base_ptx_lane_vec_final2
         base_pty_lane_vec_final2_speed = base_pty_lane_vec_final2
    else:
         muy_lane_vec_final2_speed = muy_lane_vec_final2_previous
         mux_lane_vec_final2_speed = mux_lane_vec_final2_previous
         base_ptx_lane_vec_final2_speed = base_ptx_lane_vec_final2_previous
         base_pty_lane_vec_final2_speed = base_pty_lane_vec_final2_previous

    (speed, road_nomark, capture_frameindex_for_speed,
     frameindex_for_speed, white_mark_hit, speed_read_flag,
     count_scanned_lines_reverse_for_speed) = absolute_speed_estimation(
        muy_lane_vec_final2_speed,
        mux_lane_vec_final2_speed,
        base_ptx_lane_vec_final2_speed,
        base_pty_lane_vec_final2_speed,
        h1,
        capture_frameindex_for_speed,
        frameindex_for_speed_previous,
        frameindex_for_speed,
        image_number,
        white_mark_hit,
        count_scanned_lines_reverse_for_speed_previous,
        count_scanned_lines_reverse_for_speed,
        img_subframe,
        img_subframe_gray)

    #Update Speed reading
    if speed_read_flag == 1:
        speed_official = speed
        first_reading_available_flag = 1

    if road_nomark == 1 and white_mark_hit == 1:
        count_road_nomark += 1

    # Detecting ending of white road mark (the marker is over the pavement)
    if count_road_nomark == 5:
        white_mark_hit = 0
        count_road_nomark = 0
        capture_frameindex_for_speed = 0
        frameindex_for_speed_previous = frameindex_for_speed
        count_scanned_lines_reverse_for_speed_previous = count_scanned_lines_reverse_for_speed
        prev_s = 0

    #Speed on right track (group1)
    if count_lane_group1 >= 1:
        muy_lane_vec_final1_speed = muy_lane_vec_final1
        mux_lane_vec_final1_speed = mux_lane_vec_final1
        base_ptx_lane_vec_final1_speed = base_ptx_lane_vec_final1
        base_pty_lane_vec_final1_speed = base_pty_lane_vec_final1
    else:
        muy_lane_vec_final1_speed = muy_lane_vec_final1_previous
        mux_lane_vec_final1_speed = mux_lane_vec_final1_previous
        base_ptx_lane_vec_final1_speed = base_ptx_lane_vec_final1_previous
        base_pty_lane_vec_final1_speed = base_pty_lane_vec_final1_previous

    (speed_1, road_nomark_1, capture_frameindex_for_speed_1,
     frameindex_for_speed_1, white_mark_hit_1, speed_read_flag_1,
     count_scanned_lines_reverse_for_speed_1) = absolute_speed_estimation(
        muy_lane_vec_final1_speed,
        mux_lane_vec_final1_speed,
        base_ptx_lane_vec_final1_speed,
        base_pty_lane_vec_final1_speed,
        h1,
        capture_frameindex_for_speed_1,
        frameindex_for_speed_previous_1,
        frameindex_for_speed_1,
        image_number,
        white_mark_hit_1,
        count_scanned_lines_reverse_for_speed_previous_1,
        count_scanned_lines_reverse_for_speed_1,
        img_subframe,
        img_subframe_gray)

    #Update Speed reading
    if speed_read_flag_1 == 1:
        speed_official = speed_1
        first_reading_available_flag = 1

    if road_nomark_1 == 1 and white_mark_hit_1 == 1:
        count_road_nomark_1 += 1

    # Detecting ending of white road mark (the marker is over the pavement)
    if count_road_nomark_1 == 5:
        white_mark_hit_1 = 0
        count_road_nomark_1 = 0
        capture_frameindex_for_speed_1 = 0
        frameindex_for_speed_previous_1 = frameindex_for_speed_1
        count_scanned_lines_reverse_for_speed_previous_1 = \
            count_scanned_lines_reverse_for_speed_1

    ######## RECORDING OF CURRENTLY DETECTED LANES ########
    mux_lane_vec_final2_previous = mux_lane_vec_final2
    muy_lane_vec_final2_previous = muy_lane_vec_final2
    base_ptx_lane_vec_final2_previous = base_ptx_lane_vec_final2
    base_pty_lane_vec_final2_previous = base_pty_lane_vec_final2

    mux_lane_vec_final1_previous = mux_lane_vec_final1
    muy_lane_vec_final1_previous = muy_lane_vec_final1
    base_ptx_lane_vec_final1_previous = base_ptx_lane_vec_final1
    base_pty_lane_vec_final1_previous = base_pty_lane_vec_final1

    ######## GENERATION OF LONG TERM AVERAGE OF THE DETECTED LANES ########
    (count_lanes_average_vec, count_lanes_average_vec2, mux_lane_vec_average,
     muy_lane_vec_average, base_ptx_lane_vec_average, base_pty_lane_vec_average,
     mux_lane_vec_average2, muy_lane_vec_average2, base_ptx_lane_vec_average2,
     base_pty_lane_vec_average2) = long_term_average_of_lanes(
        count_lanes_average_vec,
        count_lanes_average_vec2,
        mux_lane_vec_average,
        muy_lane_vec_average,
        base_ptx_lane_vec_average,
        base_pty_lane_vec_average,
        mux_lane_vec_average2,
        muy_lane_vec_average2,
        base_ptx_lane_vec_average2,
        base_pty_lane_vec_average2,
        mux_lane_vec_final1,
        muy_lane_vec_final1,
        base_ptx_lane_vec_final1,
        base_pty_lane_vec_final1,
        mux_lane_vec_final2,
        muy_lane_vec_final2,
        base_ptx_lane_vec_final2,
        base_pty_lane_vec_final2,
        img_subframe,
        H)

    initial_frame_was_processed_flag = 1
    ############## DISPLAY #################
    #resizing image for displaying purposes
    img7 = img_subframe
    dim = (3840, 2880)
    resized = cv2.resize(img7, dim, interpolation = cv2.INTER_CUBIC)

    if first_reading_available_flag != 0:
        print("Official speed:", speed_official)
        speed_text = 'Speed: '+str(int(speed_official))+' miles/hr'
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
