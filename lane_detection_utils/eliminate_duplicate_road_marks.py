"""
Eliminate Duplicate Road marks
Author: Juan Carlos Aragon - Allstate
Minor Editing: djp42 - Stanford
See README for more details.
"""
import math

def eliminate_duplicate_road_marks_w(lane_detector_object):
    """
    A wrapper to make this function ~relatively~ modular and work with a class.
    """
    (lane_detector_object.mux_lane_vec_aggregated, lane_detector_object.muy_lane_vec_aggregated,
     lane_detector_object.base_ptx_lane_vec_aggregated, lane_detector_object.base_pty_lane_vec_aggregated,
     lane_detector_object.angle_lanes, lane_detector_object.count_tracked_lanes2) = \
        eliminate_duplicate_road_marks(
            lane_detector_object.mux_lane_vec,
            lane_detector_object.muy_lane_vec,
            lane_detector_object.base_ptx_lane_vec,
            lane_detector_object.base_pty_lane_vec,
            lane_detector_object.count_lanes,
            lane_detector_object.mux_lane_vec_aggregated,
            lane_detector_object.muy_lane_vec_aggregated,
            lane_detector_object.base_ptx_lane_vec_aggregated,
            lane_detector_object.base_pty_lane_vec_aggregated,
            lane_detector_object.angle_lanes)

def eliminate_duplicate_road_marks(mux_lane_vec,
                                   muy_lane_vec,
                                   base_ptx_lane_vec,
                                   base_pty_lane_vec,
                                   count_lanes,
                                   mux_lane_vec_aggregated,
                                   muy_lane_vec_aggregated,
                                   base_ptx_lane_vec_aggregated,
                                   base_pty_lane_vec_aggregated,
                                   angle_lanes):

#we will elminate duplicates on the tracked lines
    count_tracked_lanes2 = 0
    if count_lanes != 0:
        #count_tracked_lanes2 = 0
        for lanes in range(0, count_lanes):
            if lanes >= 1:
                repeat = 0
                for lanes2 in range(0, lanes):
                    if mux_lane_vec[lanes2] == mux_lane_vec[lanes]:
                        repeat = 1 #we found a repeat, the repeats exist because the scanning region detects more than once
                        break
                if repeat == 0:
                    mux_lane_vec_aggregated[count_tracked_lanes2] = \
                        mux_lane_vec[lanes]
                    muy_lane_vec_aggregated[count_tracked_lanes2] = \
                        muy_lane_vec[lanes]
                    base_ptx_lane_vec_aggregated[count_tracked_lanes2] = \
                        base_ptx_lane_vec[lanes]
                    base_pty_lane_vec_aggregated[count_tracked_lanes2] = \
                        base_pty_lane_vec[lanes]
                    angle_lanes[count_tracked_lanes2] = math.atan(
                            muy_lane_vec[lanes]/mux_lane_vec[lanes]
                        ) * 180 / math.pi
                    count_tracked_lanes2 += 1
            else:
                mux_lane_vec_aggregated[count_tracked_lanes2] = \
                    mux_lane_vec[lanes]
                muy_lane_vec_aggregated[count_tracked_lanes2] = \
                    muy_lane_vec[lanes]
                base_ptx_lane_vec_aggregated[count_tracked_lanes2] = \
                    base_ptx_lane_vec[lanes]
                base_pty_lane_vec_aggregated[count_tracked_lanes2] = \
                    base_pty_lane_vec[lanes]
                angle_lanes[count_tracked_lanes2] = math.atan(
                        muy_lane_vec[lanes]/mux_lane_vec[lanes]
                    ) * 180 / math.pi
                count_tracked_lanes2 += 1

    return mux_lane_vec_aggregated, \
           muy_lane_vec_aggregated, \
           base_ptx_lane_vec_aggregated, \
           base_pty_lane_vec_aggregated, \
           angle_lanes, \
           count_tracked_lanes2
