"""
ABSOLUTE SPEED ESTIMATION

Author: Juan Carlos Aragon - Allstate
Minor Editing: djp42 - Stanford

See README for more details.
This procedure relies on a "marker" which is a small imaginary horizontal line that is used as reference to establish speed.

"""
import cv2

# TODO: make this a class
def abs_speed_wrapper(lane_detector_object):
    """
    A wrapper to make this function ~relatively~ modular and work with a class.
    """
    #Speed on left track (group2)
    if lane_detector_object.count_lane_group2 >= 1:
        lane_detector_object.muy_lane_vec_final2_speed = lane_detector_object.muy_lane_vec_final2
        lane_detector_object.mux_lane_vec_final2_speed = lane_detector_object.mux_lane_vec_final2
        lane_detector_object.base_ptx_lane_vec_final2_speed = lane_detector_object.base_ptx_lane_vec_final2
        lane_detector_object.base_pty_lane_vec_final2_speed = lane_detector_object.base_pty_lane_vec_final2
    else:
        lane_detector_object.muy_lane_vec_final2_speed = lane_detector_object.muy_lane_vec_final2_previous
        lane_detector_object.mux_lane_vec_final2_speed = lane_detector_object.mux_lane_vec_final2_previous
        lane_detector_object.base_ptx_lane_vec_final2_speed = lane_detector_object.base_ptx_lane_vec_final2_previous
        lane_detector_object.base_pty_lane_vec_final2_speed = lane_detector_object.base_pty_lane_vec_final2_previous

    (lane_detector_object.speed, lane_detector_object.road_nomark,
     lane_detector_object.capture_frameindex_for_speed,
     lane_detector_object.frameindex_for_speed,
     lane_detector_object.white_mark_hit,
     lane_detector_object.speed_read_flag,
     lane_detector_object.count_scanned_lines_reverse_for_speed) = absolute_speed_estimation(
        lane_detector_object.muy_lane_vec_final2_speed,
        lane_detector_object.mux_lane_vec_final2_speed,
        lane_detector_object.base_ptx_lane_vec_final2_speed,
        lane_detector_object.base_pty_lane_vec_final2_speed,
        lane_detector_object.h1,
        lane_detector_object.capture_frameindex_for_speed,
        lane_detector_object.frameindex_for_speed_previous,
        lane_detector_object.frameindex_for_speed,
        lane_detector_object.image_number,
        lane_detector_object.white_mark_hit,
        lane_detector_object.count_scanned_lines_reverse_for_speed_previous,
        lane_detector_object.count_scanned_lines_reverse_for_speed,
        lane_detector_object.img_subframe,
        lane_detector_object.img_subframe_gray)

    #Update Speed reading
    if lane_detector_object.speed_read_flag == 1:
        lane_detector_object.speed_official = lane_detector_object.speed
        print("official_speed:", lane_detector_object.speed_official)
        lane_detector_object.first_reading_available_flag = 1

    if lane_detector_object.road_nomark == 1 and lane_detector_object.white_mark_hit == 1:
        lane_detector_object.count_road_nomark += 1

    # Detecting ending of white road mark (the marker is over the pavement)
    if lane_detector_object.count_road_nomark == 5:
        lane_detector_object.white_mark_hit = 0
        lane_detector_object.count_road_nomark = 0
        lane_detector_object.capture_frameindex_for_speed = 0
        lane_detector_object.frameindex_for_speed_previous = lane_detector_object.frameindex_for_speed
        lane_detector_object.count_scanned_lines_reverse_for_speed_previous = lane_detector_object.count_scanned_lines_reverse_for_speed
        lane_detector_object.prev_s = 0

    #Speed on right track (group1)
    if lane_detector_object.count_lane_group1 >= 1:
        lane_detector_object.muy_lane_vec_final1_speed = lane_detector_object.muy_lane_vec_final1
        lane_detector_object.mux_lane_vec_final1_speed = lane_detector_object.mux_lane_vec_final1
        lane_detector_object.base_ptx_lane_vec_final1_speed = lane_detector_object.base_ptx_lane_vec_final1
        lane_detector_object.base_pty_lane_vec_final1_speed = lane_detector_object.base_pty_lane_vec_final1
    else:
        lane_detector_object.muy_lane_vec_final1_speed = lane_detector_object.muy_lane_vec_final1_previous
        lane_detector_object.mux_lane_vec_final1_speed = lane_detector_object.mux_lane_vec_final1_previous
        lane_detector_object.base_ptx_lane_vec_final1_speed = lane_detector_object.base_ptx_lane_vec_final1_previous
        lane_detector_object.base_pty_lane_vec_final1_speed = lane_detector_object.base_pty_lane_vec_final1_previous

    (lane_detector_object.speed_1, lane_detector_object.road_nomark_1,
     lane_detector_object.capture_frameindex_for_speed_1,
     lane_detector_object.frameindex_for_speed_1,
     lane_detector_object.white_mark_hit_1,
     lane_detector_object.speed_read_flag_1,
     lane_detector_object.count_scanned_lines_reverse_for_speed_1) = absolute_speed_estimation(
        lane_detector_object.muy_lane_vec_final1_speed,
        lane_detector_object.mux_lane_vec_final1_speed,
        lane_detector_object.base_ptx_lane_vec_final1_speed,
        lane_detector_object.base_pty_lane_vec_final1_speed,
        lane_detector_object.h1,
        lane_detector_object.capture_frameindex_for_speed_1,
        lane_detector_object.frameindex_for_speed_previous_1,
        lane_detector_object.frameindex_for_speed_1,
        lane_detector_object.image_number,
        lane_detector_object.white_mark_hit_1,
        lane_detector_object.count_scanned_lines_reverse_for_speed_previous_1,
        lane_detector_object.count_scanned_lines_reverse_for_speed_1,
        lane_detector_object.img_subframe,
        lane_detector_object.img_subframe_gray)

    #Update Speed reading
    if lane_detector_object.speed_read_flag_1 == 1:
        lane_detector_object.speed_official = lane_detector_object.speed_1
        print("official_speed:", lane_detector_object.speed_official)
        lane_detector_object.first_reading_available_flag = 1

    if lane_detector_object.road_nomark_1 == 1 and lane_detector_object.white_mark_hit_1 == 1:
        lane_detector_object.count_road_nomark_1 += 1

    # Detecting ending of white road mark (the marker is over the pavement)
    if lane_detector_object.count_road_nomark_1 == 5:
        lane_detector_object.white_mark_hit_1 = 0
        lane_detector_object.count_road_nomark_1 = 0
        lane_detector_object.capture_frameindex_for_speed_1 = 0
        lane_detector_object.frameindex_for_speed_previous_1 = lane_detector_object.frameindex_for_speed_1
        lane_detector_object.count_scanned_lines_reverse_for_speed_previous_1 = \
            lane_detector_object.count_scanned_lines_reverse_for_speed_1


def absolute_speed_estimation(muy_lane_vec_speed,
                              mux_lane_vec_speed,
                              base_ptx_lane_vec_speed,
                              base_pty_lane_vec_speed,
                              h1,
                              capture_frameindex_for_speed,
                              frameindex_for_speed_previous,
                              frameindex_for_speed,
                              image_number,
                              white_mark_hit,
                              count_scanned_lines_reverse_for_speed_previous,
                              count_scanned_lines_reverse_for_speed,
                              img6,
                              img2):
    speed_read_flag = 0
    speed = 0
    if muy_lane_vec_speed == 0:
        print("0 muy lane vec speed")
        muy_lane_vec_speed += 1e-10

    #Intersection between the Lane and the marker
    Lintersection = (0.8 * h1-base_pty_lane_vec_speed) / muy_lane_vec_speed
    x2_lane = base_ptx_lane_vec_speed + Lintersection * mux_lane_vec_speed

    cv2.line(img6,
             (int(x2_lane - 50), int(0.8 * h1)),
             (int(x2_lane + 50), int(0.8 * h1)),
             (255, 0, 0),
             1,
             cv2.LINE_AA)

    #Scanning horizontally over the marker and detect high transition
    road_nomark = 1
    for k5 in range(-15, 15):

        value = img2[int(0.8 * h1), int(x2_lane + k5)]

        if k5 > -15:
            if value > 1.2 * previous:
                road_nomark = 0
                #scanning upwards to find the end of the mark
                found_end_of_mark = 0
                count_scanned_lines = 0
                for k6 in range(0, 100):
                    Lintersection = (0.8 * h1 - k6 - base_pty_lane_vec_speed) /\
                                    muy_lane_vec_speed
                    x2_lane_scan = base_ptx_lane_vec_speed + \
                                   Lintersection * mux_lane_vec_speed
                    cv2.line(img6,
                             (int(x2_lane_scan-50), int(0.8 * h1-k6)),
                             (int(x2_lane_scan + 50), int(0.8 * h1-k6)),
                             (255, 0, 0),
                             1,
                             cv2.LINE_AA)
                    count_scanned_lines += 1
                    out_of_the_mark = 1
                    for k7 in range(-10, 10):

                        value_scan = img2[int(0.8 * h1 - k6), int(x2_lane_scan + k7)]
                        if k7 > -10:
                            if value_scan>1.2 * previous_scan:
                                out_of_the_mark = 0
                                break

                        previous_scan = value_scan

                    if out_of_the_mark == 1:
                        break

                #finding the beginning of the lane (we scan in reverse when the mark is not set properly at beggining of road mark. Reverse direction is scanning downwards)
                found_end_of_mark_reverse = 0
                count_scanned_lines_reverse = 0
                for k6 in range(0, 100):
                    Lintersection = (0.8 * h1 + k6 - base_pty_lane_vec_speed) /\
                                    muy_lane_vec_speed
                    x2_lane_scan = base_ptx_lane_vec_speed + \
                                   Lintersection * mux_lane_vec_speed
                    cv2.line(img6,
                            (int(x2_lane_scan-50), int(0.8 * h1 + k6)),
                            (int(x2_lane_scan + 50), int(0.8 * h1 + k6)),
                            (0, 0, 255),
                            1,
                            cv2.LINE_AA)
                    count_scanned_lines_reverse += 1
                    out_of_the_mark_reverse = 1
                    for k7 in range(-20, 20):
                        if 0.8 * h1 + k6 >= h1:
                            break
                        else:
                            value_scan = img2[int(0.8 * h1 + k6), int(x2_lane_scan + k7)]
                        if k7 > -20:
                            if value_scan > 1.2 * previous_scan:
                                out_of_the_mark_reverse = 0
                                break
                        previous_scan = value_scan

                    if out_of_the_mark_reverse == 1:
                        break

                if capture_frameindex_for_speed == 0 and count_scanned_lines >= 20:
                    white_mark_hit = 1
                    frameindex_for_speed = image_number
                    count_scanned_lines_reverse_for_speed = count_scanned_lines_reverse
                    capture_frameindex_for_speed = 1
                    if frameindex_for_speed_previous != 0:
                        time = (frameindex_for_speed-frameindex_for_speed_previous) / 29
                        speed = (40 / time) * 0.682
                        speed_read_flag = 1
                        if count_scanned_lines + count_scanned_lines_reverse > 40: #meaning correct road mark
                            correction_factor = (
                                count_scanned_lines_reverse / 115
                            ) * 10
                            #excess of distance traveled beyond the beginning of the white road marking. The "10" here is the 10 feet of the road marking
                            correction_factor2 = (
                                count_scanned_lines_reverse_for_speed_previous /
                                115
                            ) * 10
                            speed = (
                                (40 - correction_factor-correction_factor2) /
                                time
                            ) * 0.682
                break
        previous = value



    return speed, \
           road_nomark, \
           capture_frameindex_for_speed, \
           frameindex_for_speed, \
           white_mark_hit, \
           speed_read_flag, \
           count_scanned_lines_reverse_for_speed
