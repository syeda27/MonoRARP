import cv2

"""
Signature Detection

Author: Juan Carlos Aragon  -  Allstate
Minor Editing: djp42  -  Stanford

See README for more details.

"""
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
                             count_lanes_previous,
                             count_lanes,
                             H,
                             img6):

    signature_detected = 0

    if abs(road1_average-road2_average) / road2_average < 0.15 \
            and delta_road1_average < 10 \
            and delta_road2_average < 10:
        if abs(road3_average - road4_average) / road4_average < 0.15 \
                and delta_road3_average < 16 \
                and delta_road4_average < 16:
            if whitemarkings_average / road1_average > 1.5:
                if delta_whitemarkings_average < 50:
                    if rx1[top_left] > 1500 and rx1[top_right] < 2700:  #focusing the detection around the center of the ego-vehicle
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
                        if count_lanes_previous != 0:
                            for lanes in range(0, count_lanes_previous):
                                #intersecting with top of image
                                Lintersection = \
                                    -base_pty_lane_vec_previous[lanes] / \
                                    muy_lane_vec_previous[lanes]
                                x1_lane = base_ptx_lane_vec_previous[lanes] + \
                                    Lintersection*mux_lane_vec_previous[lanes]
                                #intersection with bottom of image
                                Lintersection = (
                                    H - base_pty_lane_vec_previous[lanes]
                                ) / muy_lane_vec_previous[lanes]
                                x2_lane = base_ptx_lane_vec_previous[lanes] + \
                                          Lintersection * \
                                          mux_lane_vec_previous[lanes]
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
