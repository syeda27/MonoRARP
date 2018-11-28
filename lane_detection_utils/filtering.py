"""
Filtering
Once the lanes have been generated we compare such lanes with previously generated lanes for the corresponding side (left or rith).

Author: Juan Carlos Aragon  -  Allstate
Minor Editing: djp42  -  Stanford

See README for more details.
"""
import math

def filtering_w(lane_detector_object):
    """
    A wrapper to make this function ~relatively~ modular and work with a class.
    """
    (lane_detector_object.mux_lane_vec_final1,
     lane_detector_object.muy_lane_vec_final1,
     lane_detector_object.base_ptx_lane_vec_final1,
     lane_detector_object.base_pty_lane_vec_final1,
     lane_detector_object.mux_lane_vec_final2,
     lane_detector_object.muy_lane_vec_final2,
     lane_detector_object.base_ptx_lane_vec_final2,
     lane_detector_object.base_pty_lane_vec_final2,
     lane_detector_object.x1_lane_group1,
     lane_detector_object.x1_lane_group2) = filtering(
        lane_detector_object.count_lane_group1,
        lane_detector_object.count_lane_group2,
        lane_detector_object.x1_lane_group1,
        lane_detector_object.x1_lane_group2,
        lane_detector_object.H,
        lane_detector_object.initial_frame_was_processed_flag,
        lane_detector_object.mux_lane_vec_final1,
        lane_detector_object.muy_lane_vec_final1,
        lane_detector_object.base_ptx_lane_vec_final1,
        lane_detector_object.base_pty_lane_vec_final1,
        lane_detector_object.mux_lane_vec_final1_previous,
        lane_detector_object.muy_lane_vec_final1_previous,
        lane_detector_object.base_ptx_lane_vec_final1_previous,
        lane_detector_object.base_pty_lane_vec_final1_previous,
        lane_detector_object.mux_lane_vec_final2,
        lane_detector_object.muy_lane_vec_final2,
        lane_detector_object.base_ptx_lane_vec_final2,
        lane_detector_object.base_pty_lane_vec_final2,
        lane_detector_object.mux_lane_vec_final2_previous,
        lane_detector_object.muy_lane_vec_final2_previous,
        lane_detector_object.base_ptx_lane_vec_final2_previous,
        lane_detector_object.base_pty_lane_vec_final2_previous,
        lane_detector_object.horizontal_tolerance)

def filtering(count_lane_group1,
              count_lane_group2,
              x1_lane_group1,
              x1_lane_group2,
              H,
              initial_frame,
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
              base_pty_lane_vec_final2_previous,
              horizontal_tolerance=50):
    if muy_lane_vec_final1_previous == 0:
        print("Filtering: muy_lane_vec_final1_previous is 0")
        muy_lane_vec_final1_previous += 1e-10
    if mux_lane_vec_final1_previous == 0:
        print("Filtering: muy_lane_vec_final1_previous is 0")
        mux_lane_vec_final1_previous += 1e-10
    if count_lane_group1 >= 1:
        if initial_frame == 1:
            angle1 = 180 * (
                math.atan(muy_lane_vec_final1 / mux_lane_vec_final1)) / 3.14159
            angle2 = 180 * (
                math.atan(muy_lane_vec_final1_previous /
                          mux_lane_vec_final1_previous)
                ) / 3.14159

            if abs(angle1 - angle2) > 10:
                #("bad lane")
                mux_lane_vec_final1 = mux_lane_vec_final1_previous
                muy_lane_vec_final1 = muy_lane_vec_final1_previous
                base_ptx_lane_vec_final1 = base_ptx_lane_vec_final1_previous
                base_pty_lane_vec_final1 = base_pty_lane_vec_final1_previous

        #intersecting with top of image
        Lintersection =  -base_pty_lane_vec_final1 / muy_lane_vec_final1
        x1_lane = base_ptx_lane_vec_final1 + Lintersection * mux_lane_vec_final1
        #intersection with bottom of image
        Lintersection = (H - base_pty_lane_vec_final1) / muy_lane_vec_final1
        x2_lane = base_ptx_lane_vec_final1 + Lintersection * mux_lane_vec_final1
        #intersection with bottom of image (for img3 the intersection needs to be different because the distance base_pty_lane_vec_final1 will be 1500 + base_pty_lane_vec_final1 and this is closer to the bottom of the image than the previous case (we are using H = 2160 in both cases, above it was a mistake still worked). So being closer to the bottom then x2_lane needs to be shorter.
        Lintersection2 = (H - (1500 + base_pty_lane_vec_final1)) / \
                         muy_lane_vec_final1
        x2_lane2 = base_ptx_lane_vec_final1 + \
                   Lintersection2 * mux_lane_vec_final1

        if initial_frame == 1:
            if abs(x1_lane_group1 - x1_lane) > horizontal_tolerance:
                mux_lane_vec_final1 = mux_lane_vec_final1_previous
                muy_lane_vec_final1 = muy_lane_vec_final1_previous
                base_ptx_lane_vec_final1 = base_ptx_lane_vec_final1_previous
                base_pty_lane_vec_final1 = base_pty_lane_vec_final1_previous
                Lintersection =  -base_pty_lane_vec_final1 / muy_lane_vec_final1
                x1_lane = base_ptx_lane_vec_final1 + \
                          Lintersection * mux_lane_vec_final1
                Lintersection = (H - base_pty_lane_vec_final1) / \
                                muy_lane_vec_final1
                x2_lane = base_ptx_lane_vec_final1 + \
                          Lintersection * mux_lane_vec_final1
        x1_lane_group1 = x1_lane
    else:
        #intersecting with top of image
        Lintersection =  -base_pty_lane_vec_final1_previous / \
                          muy_lane_vec_final1_previous
        x1_lane = base_ptx_lane_vec_final1_previous + \
                  Lintersection * mux_lane_vec_final1_previous
        #intersection with bottom of image
        Lintersection = (H - base_pty_lane_vec_final1_previous) / \
                        muy_lane_vec_final1_previous
        x2_lane = base_ptx_lane_vec_final1_previous + \
                  Lintersection * mux_lane_vec_final1_previous


        mux_lane_vec_final1 = mux_lane_vec_final1_previous
        muy_lane_vec_final1 = muy_lane_vec_final1_previous
        base_ptx_lane_vec_final1 = base_ptx_lane_vec_final1_previous
        base_pty_lane_vec_final1 = base_pty_lane_vec_final1_previous

        x1_lane_group1 = x1_lane


    if count_lane_group2 >= 1:
        if mux_lane_vec_final2_previous == 0:
            print("Filtering: mux_lane_vec_final2_previous is 0")
            mux_lane_vec_final2_previous += 1e-10
        if muy_lane_vec_final2_previous == 0:
            muy_lane_vec_final2_previous += 1e-10
        if initial_frame == 1:
            angle1 = 180 * (
                math.atan(muy_lane_vec_final2 / mux_lane_vec_final2)) / 3.14159
            angle2 = 180 * (
                math.atan(muy_lane_vec_final2_previous /
                          mux_lane_vec_final2_previous)
            ) / 3.14159
            if abs(angle1 - angle2) > 10:
                #("bad lane")
                mux_lane_vec_final2 = mux_lane_vec_final2_previous
                muy_lane_vec_final2 = muy_lane_vec_final2_previous
                base_ptx_lane_vec_final2 = base_ptx_lane_vec_final2_previous
                base_pty_lane_vec_final2 = base_pty_lane_vec_final2_previous

        #intersecting with top of image
        Lintersection =  -base_pty_lane_vec_final2 / muy_lane_vec_final2
        x1_lane = base_ptx_lane_vec_final2 + Lintersection*mux_lane_vec_final2
        #intersection with bottom of image
        Lintersection = (H - base_pty_lane_vec_final2) / muy_lane_vec_final2
        x2_lane = base_ptx_lane_vec_final2 + Lintersection*mux_lane_vec_final2
        #intersection with bottom of image (for img3 the intersection needs to be different because the distance base_pty_lane_vec_final1 will be 1500 + base_pty_lane_vec_final1 and this is closer to the bottom of the image than the previous case (we are using H = 2160 in both cases, above it was a mistake still worked). So being closer to the bottom then x2_lane needs to be shorter.
        Lintersection2 = (H - (1500 + base_pty_lane_vec_final2)) / \
                         muy_lane_vec_final2
        x2_lane2 = base_ptx_lane_vec_final2 + \
                   Lintersection2 * mux_lane_vec_final2

        if initial_frame == 1:
            if abs(x1_lane_group2 - x1_lane) > horizontal_tolerance:
                mux_lane_vec_final2 = mux_lane_vec_final2_previous
                muy_lane_vec_final2 = muy_lane_vec_final2_previous
                base_ptx_lane_vec_final2 = base_ptx_lane_vec_final2_previous
                base_pty_lane_vec_final2 = base_pty_lane_vec_final2_previous
                Lintersection =  -base_pty_lane_vec_final2 / muy_lane_vec_final2
                x1_lane = base_ptx_lane_vec_final2 + \
                          Lintersection * mux_lane_vec_final2
                Lintersection = (H - base_pty_lane_vec_final2) / \
                          muy_lane_vec_final2
                x2_lane = base_ptx_lane_vec_final2 + \
                          Lintersection * mux_lane_vec_final2
        x1_lane_group2 = x1_lane
    else:
        if muy_lane_vec_final2_previous == 0:
            print("Filtering: muy_lane_vec_final2_previous is 0")
            muy_lane_vec_final2_previous += 1e-10
        #intersecting with top of image
        Lintersection =  -base_pty_lane_vec_final2_previous / muy_lane_vec_final2_previous
        x1_lane = base_ptx_lane_vec_final2_previous + Lintersection*mux_lane_vec_final2_previous
        #intersection with bottom of image
        Lintersection = (H - base_pty_lane_vec_final2_previous) / muy_lane_vec_final2_previous
        x2_lane = base_ptx_lane_vec_final2_previous + Lintersection * mux_lane_vec_final2_previous
        mux_lane_vec_final2 = mux_lane_vec_final2_previous
        muy_lane_vec_final2 = muy_lane_vec_final2_previous
        base_ptx_lane_vec_final2 = base_ptx_lane_vec_final2_previous
        base_pty_lane_vec_final2 = base_pty_lane_vec_final2_previous

        x1_lane_group2 = x1_lane


    return mux_lane_vec_final1, \
           muy_lane_vec_final1, \
           base_ptx_lane_vec_final1, \
           base_pty_lane_vec_final1, \
           mux_lane_vec_final2, \
           muy_lane_vec_final2, \
           base_ptx_lane_vec_final2, \
           base_pty_lane_vec_final2, \
           x1_lane_group1, \
           x1_lane_group2
