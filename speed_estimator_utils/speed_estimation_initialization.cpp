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
#include "speed_estimation.h"

using namespace std;
using namespace cv;


Speed_estimator::Speed_estimator(bool display)
{
    // TODO: args


    white_mark_hit=0;
    count_road_nomark=0;
    capture_frameindex_for_speed=0;
    frameindex_for_speed_previous=0;
    frameindex_for_speed=0;
    count_scanned_lines_reverse_for_speed_previous=0;
    count_scanned_lines_reverse_for_speed=0;
    count_scanned_lines_for_speed_previous=0;
    count_scanned_lines_for_speed=0;
    road_nomark=0;


    white_mark_hit_1=0;
    count_road_nomark_1=0;
    capture_frameindex_for_speed_1=0;
    frameindex_for_speed_previous_1=0;
    frameindex_for_speed_1=0;
    count_scanned_lines_reverse_for_speed_previous_1=0;
    count_scanned_lines_reverse_for_speed_1=0;
    count_scanned_lines_for_speed_previous_1=0;
    count_scanned_lines_for_speed_1=0;
    road_nomark_1=0;

    base_ptx_lane_vec_final1=0;
    base_pty_lane_vec_final1=0;
    mux_lane_vec_final1=0;
    muy_lane_vec_final1=0;

    base_ptx_lane_vec_final2=0;
    base_pty_lane_vec_final2=0;
    mux_lane_vec_final2=0;
    muy_lane_vec_final2=0;

    mux_lane_vec_final1_previous=0;
    muy_lane_vec_final1_previous=0;
    base_ptx_lane_vec_final1_previous=0;
    base_pty_lane_vec_final1_previous=0;

    mux_lane_vec_final2_previous=0;
    muy_lane_vec_final2_previous=0;
    base_ptx_lane_vec_final2_previous=0;
    base_pty_lane_vec_final2_previous=0;


    initial_frame_was_processed_flag=0;

    first_reading_available_flag=0;

    parmanent_positioning=0;

    offset=0;
    offset_at_end_of_mark=0;
    offset_at_middle_of_mark=0;
    offset_1=0;
    offset_at_end_of_mark_1=0;
    offset_at_middle_of_mark_1=0;

    offset_adjustment=0;
    offset_adjustment_1=0;
    count_marks_detected_for_adjustment=0;
    count_marks_detected_for_adjustment_b=0;
    count_marks_detected_for_adjustment_c=0;
    count_marks_detected_for_adjustment_1=0;
    count_marks_detected_for_adjustment_b_1=0;
    count_marks_detected_for_adjustment_c_1=0;
    offset_adjustment_acc=0;
    offset_adjustment_acc_b=0;
    offset_adjustment_acc_c=0;
    offset_adjustment_acc_1=0;
    offset_adjustment_acc_b_1=0;
    offset_adjustment_acc_c_1=0;
    count_adjustments=0;
    count_adjustments_b=0;
    count_adjustments_c=0;
    count_adjustments_1=0;
    count_adjustments_b_1=0;
    count_adjustments_c_1=0;
    exclude=0;
    exclude_b=0;
    exclude_c=0;
    exclude_1=0;
    exclude_b_1=0;
    exclude_c_1=0;
    range_adjustment_left=0;
    range_adjustment_right=0;
    range_adjustment_left_1=0;
    range_adjustment_right_1=0;

    Left_lane_available_display=0;
    Right_lane_available_display=0;


}


Speed_estimator::~Speed_estimator()
{
}
