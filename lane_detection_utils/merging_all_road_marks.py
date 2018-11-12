"""
Merging all the Road Marks

Author: Juan Carlos Aragon - Allstate
Minor Editing: djp42 - Stanford

See README for more details.
Merging by averaging the base points of the white marks.
"""
import numpy as np

def merging_all_road_marks_w(lane_detector_object):
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
     lane_detector_object.count_lane_group1,
     lane_detector_object.count_lane_group2) = merging_all_road_marks(
        lane_detector_object.angle_lanes,
        lane_detector_object.count_tracked_lanes2,
        lane_detector_object.mux_lane_vec_aggregated,
        lane_detector_object.muy_lane_vec_aggregated,
        lane_detector_object.base_ptx_lane_vec_aggregated,
        lane_detector_object.base_pty_lane_vec_aggregated)

def merging_all_road_marks(angle_lanes,
                           count_tracked_lanes2,
                           mux_lane_vec_aggregated,
                           muy_lane_vec_aggregated,
                           base_ptx_lane_vec_aggregated,
                           base_pty_lane_vec_aggregated):
    #Merging all found lanes (without repetition) into two lanes
    #first pick the first lane
    print("count_tracked_lanes: ", count_tracked_lanes2)

    lane_group1 = np.zeros(40) #one group of lanes with positive angle
    lane_group2 = np.zeros(40) #one group of lanes with negative angle

    count_lane_group1 = 0
    count_lane_group2 = 0

    mux_lane_vec_final1 = 0
    muy_lane_vec_final1 = 0
    base_ptx_lane_vec_final1 = 0
    base_pty_lane_vec_final1 = 0

    mux_lane_vec_final2 = 0
    muy_lane_vec_final2 = 0
    base_ptx_lane_vec_final2 = 0
    base_pty_lane_vec_final2 = 0

    for lanes in range(0, count_tracked_lanes2):
        if angle_lanes[lanes] > 0:
            lane_group1[count_lane_group1] = int(lanes) #storing indexes
            count_lane_group1 += 1

        else:
            lane_group2[count_lane_group2] = int(lanes)
            count_lane_group2 += 1

    if count_lane_group1 >= 2:
        mux_lane_vec_acc = 0
        muy_lane_vec_acc = 0
        base_ptx_lane_vec_acc = 0
        base_pty_lane_vec_acc = 0
        for k4 in range(0, count_lane_group1):
            mux_lane_vec_acc = mux_lane_vec_acc + \
                               mux_lane_vec_aggregated[int(lane_group1[k4])]
            muy_lane_vec_acc = muy_lane_vec_acc + \
                               muy_lane_vec_aggregated[int(lane_group1[k4])]
            base_ptx_lane_vec_acc = base_ptx_lane_vec_acc + \
                                    base_ptx_lane_vec_aggregated[
                                        int(lane_group1[k4])
                                    ]
            base_pty_lane_vec_acc = base_pty_lane_vec_acc + \
                                    base_pty_lane_vec_aggregated[
                                        int(lane_group1[k4])
                                    ]
        mux_lane_vec_final1 = mux_lane_vec_acc / count_lane_group1
        muy_lane_vec_final1 = muy_lane_vec_acc / count_lane_group1
        base_ptx_lane_vec_final1 = base_ptx_lane_vec_acc / count_lane_group1
        base_pty_lane_vec_final1 = base_pty_lane_vec_acc / count_lane_group1
    elif count_lane_group1 == 1:
        mux_lane_vec_final1 = mux_lane_vec_aggregated[int(lane_group1[0])]
        muy_lane_vec_final1 = muy_lane_vec_aggregated[int(lane_group1[0])]
        base_ptx_lane_vec_final1 = base_ptx_lane_vec_aggregated[
            int(lane_group1[0])
        ]
        base_pty_lane_vec_final1 = base_pty_lane_vec_aggregated[
            int(lane_group1[0])
        ]
    if count_lane_group2 >= 2:
        mux_lane_vec_acc = 0
        muy_lane_vec_acc = 0
        base_ptx_lane_vec_acc = 0
        base_pty_lane_vec_acc = 0
        for k4 in range(0, count_lane_group2):
            mux_lane_vec_acc = mux_lane_vec_acc + \
                               mux_lane_vec_aggregated[int(lane_group2[k4])]
            muy_lane_vec_acc = muy_lane_vec_acc + \
                               muy_lane_vec_aggregated[int(lane_group2[k4])]
            base_ptx_lane_vec_acc = base_ptx_lane_vec_acc + \
                                    base_ptx_lane_vec_aggregated[
                                        int(lane_group2[k4])
                                    ]
            base_pty_lane_vec_acc = base_pty_lane_vec_acc + \
                                    base_pty_lane_vec_aggregated[
                                        int(lane_group2[k4])
                                    ]
        mux_lane_vec_final2 = mux_lane_vec_acc / count_lane_group2
        muy_lane_vec_final2 = muy_lane_vec_acc / count_lane_group2
        base_ptx_lane_vec_final2 = base_ptx_lane_vec_acc / count_lane_group2
        base_pty_lane_vec_final2 = base_pty_lane_vec_acc / count_lane_group2
    elif count_lane_group2 == 1:
        mux_lane_vec_final2 = mux_lane_vec_aggregated[int(lane_group2[0])]
        muy_lane_vec_final2 = muy_lane_vec_aggregated[int(lane_group2[0])]
        base_ptx_lane_vec_final2 = base_ptx_lane_vec_aggregated[
            int(lane_group2[0])
        ]
        base_pty_lane_vec_final2 = base_pty_lane_vec_aggregated[
            int(lane_group2[0])
        ]

    return mux_lane_vec_final1, \
           muy_lane_vec_final1, \
           base_ptx_lane_vec_final1, \
           base_pty_lane_vec_final1, \
           mux_lane_vec_final2, \
           muy_lane_vec_final2, \
           base_ptx_lane_vec_final2, \
           base_pty_lane_vec_final2, \
           count_lane_group1, \
           count_lane_group2
