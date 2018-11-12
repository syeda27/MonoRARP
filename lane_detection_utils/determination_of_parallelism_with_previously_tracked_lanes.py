"""
Determination of Paralelism and Distance
Author: Juan Carlos Aragon - Allstate
Minor Editing: djp42 - Stanford

See README for more details.
We assess in this function wheather or not the white marking is sufficiently parallel to a previous tracked Lane.
"""
import math

def determination_of_parallelism_w(lane_detector_object, scan_args, top_left):
    """
    A wrapper to make this function ~relatively~ modular and work with a class.
    """
    lane_detector_object.aligned_to_tracked_lane = determination_of_parallelism(
        top_left,
        scan_args.rx1,
        scan_args.ry1,
        lane_detector_object.count_lane_group1,
        lane_detector_object.count_lane_group2,
        lane_detector_object.whitemarkings_average,
        lane_detector_object.road1_average,
        lane_detector_object.base_ptx_lane_vec_final1,
        lane_detector_object.base_pty_lane_vec_final1,
        lane_detector_object.mux_lane_vec_final1,
        lane_detector_object.muy_lane_vec_final1,
        lane_detector_object.base_ptx_lane_vec_final2,
        lane_detector_object.base_pty_lane_vec_final2,
        lane_detector_object.mux_lane_vec_final2,
        lane_detector_object.muy_lane_vec_final2,
        scan_args.angles)

def determination_of_parallelism(top_left,
                                 rx1,
                                 ry1,
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
                                 angles):

    aligned_to_tracked_lane = 0

    if whitemarkings_average / road1_average > 1:
        if count_lane_group1 != 0 or count_lane_group2 != 0:

            for selection in range(0, 2): #we compare against just two previous final lanes

                proceed = 0
                if selection == 0 and count_lane_group1 != 0:
                    proceed = 1
                    xb = base_ptx_lane_vec_final1
                    yb = base_pty_lane_vec_final1
                    mux_previous_lane = mux_lane_vec_final1
                    muy_previous_lane = muy_lane_vec_final1

                if selection == 1 and count_lane_group2 != 0:
                    proceed = 1
                    xb = base_ptx_lane_vec_final2
                    yb = base_pty_lane_vec_final2
                    mux_previous_lane = mux_lane_vec_final2
                    muy_previous_lane = muy_lane_vec_final2



                if proceed == 1:
                    if abs(
                            angles[top_left] - 180 * (
                                math.atan(muy_previous_lane / mux_previous_lane)
                                / 3.14159
                                )
                            ) < 6:
                        x0 = rx1[top_left]
                        y0 = ry1[top_left]
                        Lproj = ((x0 - xb) * mux_previous_lane +
                                 (y0 - yb) * muy_previous_lane
                                 ) / \
                                 ((mux_previous_lane)**2 +
                                  (muy_previous_lane)**2
                                 )

                        xp = xb + Lproj * mux_previous_lane
                        yp = yb + Lproj * muy_previous_lane

                        distance_to_Lane = ((x0 - xp)**2 + (y0 - yp)**2)**0.5

                        if distance_to_Lane < 10:
                            aligned_to_tracked_lane = 1

    return aligned_to_tracked_lane
