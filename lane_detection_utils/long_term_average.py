"""
Long Term Average
We perform averaging of the unit vectors and of the coordinates of the base ponts for the previous average_window lanes detections performed

Author: Juan Carlos Aragon  -  Allstate
Minor Editing: djp42  -  Stanford

See README for more details.
"""
import cv2

def long_term_average_of_lanes_w(lane_detector_object):
    (lane_detector_object.count_lanes_average_vec,
     lane_detector_object.count_lanes_average_vec2,
     lane_detector_object.mux_lane_vec_average,
     lane_detector_object.muy_lane_vec_average,
     lane_detector_object.base_ptx_lane_vec_average,
     lane_detector_object.base_pty_lane_vec_average,
     lane_detector_object.mux_lane_vec_average2,
     lane_detector_object.muy_lane_vec_average2,
     lane_detector_object.base_ptx_lane_vec_average2,
     lane_detector_object.base_pty_lane_vec_average2,
     left_lane_points,
     right_lane_points) = long_term_average_of_lanes(
        lane_detector_object.count_lanes_average_vec,
        lane_detector_object.count_lanes_average_vec2,
        lane_detector_object.mux_lane_vec_average,
        lane_detector_object.muy_lane_vec_average,
        lane_detector_object.base_ptx_lane_vec_average,
        lane_detector_object.base_pty_lane_vec_average,
        lane_detector_object.mux_lane_vec_average2,
        lane_detector_object.muy_lane_vec_average2,
        lane_detector_object.base_ptx_lane_vec_average2,
        lane_detector_object.base_pty_lane_vec_average2,
        lane_detector_object.mux_lane_vec_final1,
        lane_detector_object.muy_lane_vec_final1,
        lane_detector_object.base_ptx_lane_vec_final1,
        lane_detector_object.base_pty_lane_vec_final1,
        lane_detector_object.mux_lane_vec_final2,
        lane_detector_object.muy_lane_vec_final2,
        lane_detector_object.base_ptx_lane_vec_final2,
        lane_detector_object.base_pty_lane_vec_final2,
        lane_detector_object.img_subframe,
        lane_detector_object.H,
        lane_detector_object.count_lane_group1,
        lane_detector_object.count_lane_group2,
        lane_detector_object.average_window)
    return left_lane_points, right_lane_points

def long_term_average_of_lanes(count_lanes_average_vec,
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
                               img6,
                               H,
                               count_lane_group1,
                               count_lane_group2,
                               average_window=6):
    if count_lane_group1 > 0:
        mux_lane_vec_average[count_lanes_average_vec] = mux_lane_vec_final1
        muy_lane_vec_average[count_lanes_average_vec] = muy_lane_vec_final1
        base_ptx_lane_vec_average[count_lanes_average_vec] = base_ptx_lane_vec_final1
        base_pty_lane_vec_average[count_lanes_average_vec] = base_pty_lane_vec_final1
        count_lanes_average_vec += 1
    if count_lane_group2 > 0:
        mux_lane_vec_average2[count_lanes_average_vec2] = mux_lane_vec_final2
        muy_lane_vec_average2[count_lanes_average_vec2] = muy_lane_vec_final2
        base_ptx_lane_vec_average2[count_lanes_average_vec2] = \
            base_ptx_lane_vec_final2
        base_pty_lane_vec_average2[count_lanes_average_vec2] = \
            base_pty_lane_vec_final2
        count_lanes_average_vec2 += 1

    left_lane_points = []
    right_lane_points = []

    if count_lanes_average_vec >= average_window:
        # right lane line
        mux_lane_acc = 0
        muy_lane_acc = 0
        base_ptx_acc = 0
        base_pty_acc = 0
        for k8 in range(0, average_window):
            mux_lane_acc = mux_lane_acc +  mux_lane_vec_average[
                count_lanes_average_vec - 1 - k8
            ]
            muy_lane_acc = muy_lane_acc + muy_lane_vec_average[
                count_lanes_average_vec - 1 - k8
            ]
            base_ptx_acc = base_ptx_acc + base_ptx_lane_vec_average[
                count_lanes_average_vec - 1 - k8
            ]
            base_pty_acc = base_pty_acc + base_pty_lane_vec_average[
                count_lanes_average_vec - 1 - k8
            ]
        mux_lane_ave = mux_lane_acc / average_window
        muy_lane_ave = muy_lane_acc / average_window
        base_ptx_ave = base_ptx_acc / average_window
        base_pty_ave = base_pty_acc / average_window


        #intersecting with top of image
        Lintersection = -base_pty_ave / muy_lane_ave
        x1_lane = base_ptx_ave + Lintersection * mux_lane_ave
        #intersection with bottom of image
        Lintersection = (H - base_pty_ave) / muy_lane_ave
        x2_lane = base_ptx_ave + Lintersection * mux_lane_ave

        # TOP HALF
        '''
        cv2.line(img6,
                 (int(base_ptx_ave), int(base_pty_ave)),
                 (int(x1_lane), int(0)),
                 (255, 255, 255),
                 1,
                 cv2.LINE_AA)
        # BOTTOM HALF
        cv2.line(img6,
                 (int(base_ptx_ave), int(base_pty_ave)),
                 (int(x2_lane), int(H)),
                 (255, 255, 255),
                 1,
                 cv2.LINE_AA)
        '''
        
        left_lane_points.append([
            (int(base_ptx_ave), int(base_pty_ave)),
            (int(x1_lane), int(0))
        ])
        right_lane_points.append([
             (int(base_ptx_ave), int(base_pty_ave)),
             (int(x2_lane), int(H)),
        ])

    if count_lanes_average_vec2 >= average_window:
        # LEFT LANE LINE
        mux_lane_acc = 0
        muy_lane_acc = 0
        base_ptx_acc = 0
        base_pty_acc = 0
        for k8 in range(0, average_window):
            mux_lane_acc = mux_lane_acc + mux_lane_vec_average2[
                count_lanes_average_vec2 - 1 - k8
            ]
            muy_lane_acc = muy_lane_acc + muy_lane_vec_average2[
                count_lanes_average_vec2 - 1 - k8
            ]
            base_ptx_acc = base_ptx_acc + base_ptx_lane_vec_average2[
                count_lanes_average_vec2 - 1 - k8
            ]
            base_pty_acc = base_pty_acc + base_pty_lane_vec_average2[
                count_lanes_average_vec2 - 1 - k8
            ]
        mux_lane_ave2 = mux_lane_acc / average_window
        muy_lane_ave2 = muy_lane_acc / average_window
        base_ptx_ave2 = base_ptx_acc / average_window
        base_pty_ave2 = base_pty_acc / average_window

        #intersecting with top of image
        Lintersection  =  -base_pty_ave2 / muy_lane_ave2
        x1_lane = base_ptx_ave2 + Lintersection * mux_lane_ave2
        #intersection with bottom of image
        Lintersection = (H - base_pty_ave2) / muy_lane_ave2
        x2_lane = base_ptx_ave2 + Lintersection * mux_lane_ave2

        left_lane_points.append([
                 (base_ptx_ave2, base_pty_ave2),
                 (x1_lane, 0),
        ])
        right_lane_points.append([
                 (base_ptx_ave2, base_pty_ave2),
                 (x2_lane, H),
        ])
        print(img6.shape)
        print("og left:")
        print(base_ptx_ave2, base_pty_ave2)
        print(x1_lane, 0)
        '''
        # TOP HALF
        cv2.line(img6,
                 (int(base_ptx_ave2), int(base_pty_ave2)),
                 (int(x1_lane), int(0)),
                 (255, 255, 255),
                 1,
                 cv2.LINE_AA)
        # BOTTOM HALF
        cv2.line(img6,
                 (int(base_ptx_ave2), int(base_pty_ave2)),
                 (int(x2_lane), int(H)),
                 (255, 255, 255),
                 1,
                 cv2.LINE_AA)
        '''


    return count_lanes_average_vec, \
           count_lanes_average_vec2, \
           mux_lane_vec_average, \
           muy_lane_vec_average, \
           base_ptx_lane_vec_average, \
           base_pty_lane_vec_average, \
           mux_lane_vec_average2, \
           muy_lane_vec_average2, \
           base_ptx_lane_vec_average2, \
           base_pty_lane_vec_average2, \
           left_lane_points, \
           right_lane_points,
