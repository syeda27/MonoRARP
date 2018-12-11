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

using namespace std;
using namespace cv;


class Speed_estimator
{
    public:

    void Speed_estimator_update(string, int image_number);
    void Speed_estimator_update(Mat img, double image_time);

    void absolute_speed_estimation(double &speed,int &road_nomark,int &capture_frameindex_for_speed,int &frameindex_for_speed,int &white_mark_hit,int &speed_read_flag,int &count_scanned_lines_reverse_for_speed,int &count_scanned_lines_for_speed,int &offset,int &offset_at_end_of_mark,int &offset_at_middle_of_mark,double muy_lane_vec_speed,double mux_lane_vec_speed,double base_ptx_lane_vec_speed,double base_pty_lane_vec_speed,int h1,int frameindex_for_speed_previous,int image_number,int count_scanned_lines_reverse_for_speed_previous,int count_scanned_lines_for_speed_previous,double offset_adjustment,int range_adjustment_left,int range_adjustment_right,Mat &img6,Mat &img2);
    void absolute_speed_estimation_time(double &speed,int &road_nomark,int &capture_frameindex_for_speed,double &frametime,int &white_mark_hit,int &speed_read_flag,int &count_scanned_lines_reverse_for_speed,int &count_scanned_lines_for_speed,int &offset,int &offset_at_end_of_mark,int &offset_at_middle_of_mark,double muy_lane_vec_speed,double mux_lane_vec_speed,double base_ptx_lane_vec_speed,double base_pty_lane_vec_speed,int h1,double frametime_previous,double image_time,int count_scanned_lines_reverse_for_speed_previous,int count_scanned_lines_for_speed_previous,double offset_adjustment,int range_adjustment_left,int range_adjustment_right,Mat &img6,Mat &img2);

    double get_speed();

    Speed_estimator();
    ~Speed_estimator();

    //string file_name;
    string speed_text;

    int h1;
    int w1;

    int white_mark_hit;
    int count_road_nomark;
    int capture_frameindex_for_speed;
    int frameindex_for_speed_previous;
    int frameindex_for_speed;
    double frametime_previous = 0;
    double frametime = 0;
    int count_scanned_lines_reverse_for_speed_previous;
    int count_scanned_lines_reverse_for_speed;
    int count_scanned_lines_for_speed_previous;
    int count_scanned_lines_for_speed;
    int road_nomark;


    int white_mark_hit_1;
    int count_road_nomark_1;
    int capture_frameindex_for_speed_1;
    int frameindex_for_speed_previous_1;
    int frameindex_for_speed_1;
    double frametime_previous_1 = 0;
    double frametime_1 = 0;
    int count_scanned_lines_reverse_for_speed_previous_1;
    int count_scanned_lines_reverse_for_speed_1;
    int count_scanned_lines_for_speed_previous_1;
    int count_scanned_lines_for_speed_1;
    int road_nomark_1;

    double base_ptx_lane_vec_final1;
    double base_pty_lane_vec_final1;
    double mux_lane_vec_final1;
    double muy_lane_vec_final1;

    double base_ptx_lane_vec_final2;
    double base_pty_lane_vec_final2;
    double mux_lane_vec_final2;
    double muy_lane_vec_final2;

    double mux_lane_vec_final1_previous;
    double muy_lane_vec_final1_previous;
    double base_ptx_lane_vec_final1_previous;
    double base_pty_lane_vec_final1_previous;

    double mux_lane_vec_final2_previous;
    double muy_lane_vec_final2_previous;
    double base_ptx_lane_vec_final2_previous;
    double base_pty_lane_vec_final2_previous;

    double muy_lane_vec_final2_speed_permanent_34;
    double mux_lane_vec_final2_speed_permanent_34;
    double base_ptx_lane_vec_final2_speed_permanent_34;
    double base_pty_lane_vec_final2_speed_permanent_34;

    double muy_lane_vec_final2_speed_permanent_36;
    double mux_lane_vec_final2_speed_permanent_36;
    double base_ptx_lane_vec_final2_speed_permanent_36;
    double base_pty_lane_vec_final2_speed_permanent_36;

    double muy_lane_vec_final2_speed_permanent;
    double mux_lane_vec_final2_speed_permanent;
    double base_ptx_lane_vec_final2_speed_permanent;
    double base_pty_lane_vec_final2_speed_permanent;

    double muy_lane_vec_final1_speed_permanent_34;
    double mux_lane_vec_final1_speed_permanent_34;
    double base_ptx_lane_vec_final1_speed_permanent_34;
    double base_pty_lane_vec_final1_speed_permanent_34;

    double muy_lane_vec_final1_speed_permanent_36;
    double mux_lane_vec_final1_speed_permanent_36;
    double base_ptx_lane_vec_final1_speed_permanent_36;
    double base_pty_lane_vec_final1_speed_permanent_36;

    double muy_lane_vec_final1_speed_permanent;
    double mux_lane_vec_final1_speed_permanent;
    double base_ptx_lane_vec_final1_speed_permanent;
    double base_pty_lane_vec_final1_speed_permanent;



    int initial_frame_was_processed_flag;

    int first_reading_available_flag;

    int parmanent_positioning;

    int offset;
    int offset_at_end_of_mark;
    int offset_at_middle_of_mark;
    int offset_1;
    int offset_at_end_of_mark_1;
    int offset_at_middle_of_mark_1;

    double offset_adjustment;
    double offset_adjustment_1;
    int count_marks_detected_for_adjustment;
    int count_marks_detected_for_adjustment_b;
    int count_marks_detected_for_adjustment_c;
    int count_marks_detected_for_adjustment_1;
    int count_marks_detected_for_adjustment_b_1;
    int count_marks_detected_for_adjustment_c_1;
    int offset_adjustment_acc;
    int offset_adjustment_acc_b;
    int offset_adjustment_acc_c;
    int offset_adjustment_acc_1;
    int offset_adjustment_acc_b_1;
    int offset_adjustment_acc_c_1;
    int count_adjustments;
    int count_adjustments_b;
    int count_adjustments_c;
    int count_adjustments_1;
    int count_adjustments_b_1;
    int count_adjustments_c_1;
    int exclude;
    int exclude_b;
    int exclude_c;
    int exclude_1;
    int exclude_b_1;
    int exclude_c_1;
    int range_adjustment_left;
    int range_adjustment_right;
    int range_adjustment_left_1;
    int range_adjustment_right_1;

    int Left_lane_available_display;
    int Right_lane_available_display;

    int speed_read_flag;
    double speed;
    int speed_read_flag_1;
    double speed_1;
    double speed_official;

    int any;

};
