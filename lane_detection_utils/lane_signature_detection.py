import cv2

"""
Signature Detection

Author: Juan Carlos Aragon  -  Allstate
Minor Editing: djp42  -  Stanford

See README for more details.

"""

def lane_signature_detection_w(lane_detector_object, scan_args, top_left, top_right):
    """
    A wrapper to make this function ~relatively~ modular and work with a class.
    """
    (lane_detector_object.lane_signature_detected,
     lane_detector_object.mux_lane_vec,
     lane_detector_object.muy_lane_vec,
     lane_detector_object.base_ptx_lane_vec,
     lane_detector_object.base_pty_lane_vec,
     lane_detector_object.count_lanes) = lane_signature_detection(
        lane_detector_object.road1_average,
        lane_detector_object.road2_average,
        lane_detector_object.road3_average,
        lane_detector_object.road4_average,
        lane_detector_object.delta_road1_average,
        lane_detector_object.delta_road2_average,
        lane_detector_object.delta_road3_average,
        lane_detector_object.delta_road4_average,
        lane_detector_object.whitemarkings_average,
        lane_detector_object.delta_whitemarkings_average,
        scan_args.rx1,
        scan_args.rx2,
        scan_args.ry1,
        scan_args.ry2,
        top_left,
        top_right,
        lane_detector_object.mux_lane_vec,
        lane_detector_object.muy_lane_vec,
        lane_detector_object.base_ptx_lane_vec,
        lane_detector_object.base_pty_lane_vec,
        lane_detector_object.count_lanes,
        lane_detector_object.H,
        lane_detector_object.img_subframe,
        lane_detector_object.brightness_ratio_threshold,
        lane_detector_object.left_margin_detection,
        lane_detector_object.right_margin_detection)

def lane_signature_detection(road1_average,
                             road2_average,
                             road3_average,
                             road4_average,
                             delta_road1_average,
                             delta_road2_average,
                             delta_road3_average,
                             delta_road4_average,
                             whitemarkings_average,
                             delta_whitemarkings_average,
                             rx1,
                             rx2,
                             ry1,
                             ry2,
                             top_left,
                             top_right,
                             mux_lane_vec,
                             muy_lane_vec,
                             base_ptx_lane_vec,
                             base_pty_lane_vec,
                             count_lanes,
                             H,
                             img6,
                             brightness_ratio_threshold=1.5,
                             left_margin_detection=1500,
                             right_margin_detection=2700):

    signature_detected = 0

    if abs(road1_average-road2_average) / road2_average < 0.15 \
            and delta_road1_average < 10 \
            and delta_road2_average < 10:
        if abs(road3_average - road4_average) / road4_average < 0.15 \
                and delta_road3_average < 16 \
                and delta_road4_average < 16:
            if whitemarkings_average / road1_average > brightness_ratio_threshold:
                if delta_whitemarkings_average < 50:
                    if rx1[top_left] > left_margin_detection \
                            and rx1[top_right] < right_margin_detection:
                        #focusing the detection around the center of the ego-vehicle
                        print("Lane Detected")
                        signature_detected = 1
                        L_lane = (
                            (rx1[top_left]-rx2[top_left])**2 + \
                            (ry1[top_left]-ry2[top_left])**2
                        )**0.5
                        mux_lane = (rx1[top_left] - rx2[top_left]) / L_lane
                        muy_lane = (ry1[top_left] - ry2[top_left]) / L_lane
                        #intersecting with top of image
                        Lintersection = -ry1[top_left] / muy_lane
                        x1_lane = rx1[top_left] + Lintersection*mux_lane
                        #intersection with bottom of image
                        Lintersection = (H-ry1[top_left]) / muy_lane
                        x2_lane = rx1[top_left] + Lintersection*mux_lane
                        cv2.line(img6,
                                 (int(rx1[top_left]), int(ry1[top_left])),
                                 (int(rx2[top_left]), int(ry2[top_left])),
                                 (0, 0, 255),
                                 1,
                                 cv2.LINE_AA)
                        cv2.line(img6,
                                 (int(rx1[top_right]), int(ry1[top_right])),
                                 (int(rx2[top_right]), int(ry2[top_right])),
                                 (0, 255, 0),
                                 1,
                                 cv2.LINE_AA)
                        mux_lane_vec[count_lanes] = mux_lane
                        muy_lane_vec[count_lanes] = muy_lane
                        base_ptx_lane_vec[count_lanes] = rx1[top_left]
                        base_pty_lane_vec[count_lanes] = ry1[top_left]
                        count_lanes += 1

    return signature_detected, \
           mux_lane_vec, \
           muy_lane_vec, \
           base_ptx_lane_vec, \
           base_pty_lane_vec, \
           count_lanes
