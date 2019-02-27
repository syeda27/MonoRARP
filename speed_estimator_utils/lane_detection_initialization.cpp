#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <math.h>
#include "iostream"
#include <vector>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <string>
#include <random>
#include "lane_detection.h"

using namespace std;
using namespace cv;

Lane_detector::Lane_detector()
{

    for (int k=0; k<200; k++)
    {
        rx1.push_back(0);
        rx2.push_back(0);
        ry1.push_back(0);
        ry2.push_back(0);
        angles.push_back(0);

        road1_vec.push_back(0);
        road3_vec.push_back(0);    
        road2_vec.push_back(0);
        road4_vec.push_back(0);
        lane_group1.push_back(0);
        lane_group2.push_back(0);
        
        whitemarkings_vec.push_back(0);

        
        
        
    }


    count_angles_per_region=0;

    ls = createLineSegmentDetector(LSD_REFINE_STD);

    count_lane_group1=0;
    count_lane_group2=0;
    base_ptx_lane_vec_final1=0;
    base_pty_lane_vec_final1=0;
    mux_lane_vec_final1=0;
    muy_lane_vec_final1=0;
    base_ptx_lane_vec_final2=0;
    base_pty_lane_vec_final2=0;
    mux_lane_vec_final2=0;
    muy_lane_vec_final2=0;

    for (int k=0; k<80; k++)
    {
        mux_lane_vec.push_back(0);
        muy_lane_vec.push_back(0);
        base_ptx_lane_vec.push_back(0);
        base_pty_lane_vec.push_back(0);
        angle_lanes.push_back(0);

        mux_lane_vec_aggregated.push_back(0);
        muy_lane_vec_aggregated.push_back(0);
        base_ptx_lane_vec_aggregated.push_back(0);
        base_pty_lane_vec_aggregated.push_back(0);

        mux_lane_vec_aggregated_previous.push_back(0);
        muy_lane_vec_aggregated_previous.push_back(0);
        base_ptx_lane_vec_aggregated_previous.push_back(0);
        base_pty_lane_vec_aggregated_previous.push_back(0);

        //lane_group1_b.push_back(0);

        lane_group1_previous.push_back(0);
        lane_group2_previous.push_back(0);

        //newvariable.push_back(0);

    }

    count_lanes_previous=0;

    initial_frame=0;

    mux_lane_vec_final1_previous=0;
    muy_lane_vec_final1_previous=0;
    base_ptx_lane_vec_final1_previous=0;
    base_pty_lane_vec_final1_previous=0;

    mux_lane_vec_final2_previous=0;
    muy_lane_vec_final2_previous=0;
    base_ptx_lane_vec_final2_previous=0;
    base_pty_lane_vec_final2_previous=0;

    x1_lane_group1=0;
    x1_lane_group2=0;

    count_lanes_average_vec=0;
    count_lanes_average_vec2=0;

    for (int k=0; k<16000; k++)
    {
        mux_lane_vec_average.push_back(0);
        muy_lane_vec_average.push_back(0);
        base_ptx_lane_vec_average.push_back(0);
        base_pty_lane_vec_average.push_back(0);
        mux_lane_vec_average2.push_back(0);
        muy_lane_vec_average2.push_back(0);
        base_ptx_lane_vec_average2.push_back(0);
        base_pty_lane_vec_average2.push_back(0);
    }

    count_lane_group1_previous=0;
    count_lane_group2_previous=0;

    first_reading_available_flag=0;
    road_nomark=0;
    road_nomark_1=0;
    white_mark_hit=0;
    white_mark_hit_1=0;
    count_road_nomark=0;
    count_road_nomark_1=0;
    capture_frameindex_for_speed=0;
    capture_frameindex_for_speed_1=0;
    frameindex_for_speed=0;
    frameindex_for_speed_1=0;
    frameindex_for_speed_previous=0;
    frameindex_for_speed_previous_1=0;
    count_scanned_lines_reverse_for_speed_previous=0;
    count_scanned_lines_for_speed_previous=0;
    count_scanned_lines_reverse_for_speed_previous_1=0;
    count_scanned_lines_for_speed_previous_1=0;
    count_scanned_lines_reverse_for_speed=0;
    count_scanned_lines_for_speed=0;
    count_scanned_lines_reverse_for_speed_1=0;
    count_scanned_lines_for_speed_1=0;

    speed_previous=-1;
    speed_1_previous=-1;
    speed_hold=0;
    speed_1_hold=0;
    count_hold=0;
    count_1_hold=0;

    speed_hold_due_to_delta_official=0;
    count_hold_due_to_delta_official=0;
    speed_1_hold_due_to_delta_official=0;
    count_1_hold_due_to_delta_official=0;

    speed_latest=0;
    speed_1_latest=0;
    limit=0;


    image_number=0;

    time_stamp_for_speed=0;
    time_stamp_for_speed_1=0;
    time_stamp_for_speed_previous=0;
    time_stamp_for_speed_previous_1=0;

}
