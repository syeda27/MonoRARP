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


class Lane_detector
{

    public:

    void speed_estimator_update(Mat image_color, double time);

    double get_speed();

    int scan_region(int xregion,int yregion,double region_lenght,double region_width);

    void image_reading(Mat img_color);

    int criteria_before_sampling();

    void brightness_sampling();

    void outlayer_removal_and_average_brightness_computation();

    void average_delta();

    int determination_of_parallelism();

    int lane_signature_detection(double brightness_ratio_threshold, int Left_marging_detection, int Right_margin_detection);

    void eliminate_duplicate_road_marks();

    void merging_all_road_marks();

    void filtering(double horizontal_tolerance);

    void recording_of_currently_detected_lanes();

    void long_term_average_of_lanes(int average_window,double &muy_lane_vec_final2_speed_permanent,double &mux_lane_vec_final2_speed_permanent,double &base_ptx_lane_vec_final2_speed_permanent,double &base_pty_lane_vec_final2_speed_permanent,double &muy_lane_vec_final1_speed_permanent,double &mux_lane_vec_final1_speed_permanent,double &base_ptx_lane_vec_final1_speed_permanent,double &base_pty_lane_vec_final1_speed_permanent,int &count_lanes_average_vec2_out,int &count_lanes_average_vec2_out2);

    void absolute_speed_estimation(double &speed,int &road_nomark,int &capture_frameindex_for_speed,int &frameindex_for_speed,int &white_mark_hit,int &speed_read_flag,int &count_scanned_lines_reverse_for_speed,int &count_scanned_lines_for_speed,int &offset,int &offset_at_end_of_mark,int &offset_at_middle_of_mark,double muy_lane_vec_speed,double mux_lane_vec_speed,double base_ptx_lane_vec_speed,double base_pty_lane_vec_speed,int frameindex_for_speed_previous,int image_number,int count_scanned_lines_reverse_for_speed_previous,int count_scanned_lines_for_speed_previous,double offset_adjustment,int range_adjustment_left,int range_adjustment_right, double time, double &time_stamp_for_speed, double time_stamp_for_speed_previous);    

    void display(double speed_official,int first_reading_available_flag,int image_number);

    Lane_detector();
    ~Lane_detector();




    vector<Vec4f> lines_std;

    Ptr<LineSegmentDetector> ls;

    Mat img;
    Mat img3;
    Mat img2; 
    Mat img4;
    Mat img6;
    Mat img7;

    vector<int> rx1;
    vector<int> rx2;
    vector<int> ry1;
    vector<int> ry2;

    vector<int> lane_group1;
    vector<int> lane_group2;

    vector<double> angles;

    int h1;
    int w1;
    int H;
    int W;
    int count_angles_per_region;
    int count_lanes;

    int top_left;
    int top_right;
    int counter_scanning;

    vector<double> road1_vec;
    vector<double> road3_vec;       
    vector<double> road2_vec;
    vector<double> road4_vec;
    vector<double> whitemarkings_vec;

    double road1_average;
    double road2_average;
    double road3_average;
    double road4_average;
    double whitemarkings_average;
    int removed3;
    int removed4;

    double delta_road1_average;
    double delta_road2_average;
    double delta_road3_average;
    double delta_road4_average;
    double delta_whitemarkings_average;

    int count_lane_group1;
    int count_lane_group2;
    double base_ptx_lane_vec_final1;
    double base_pty_lane_vec_final1;
    double mux_lane_vec_final1;
    double muy_lane_vec_final1;
    double base_ptx_lane_vec_final2;
    double base_pty_lane_vec_final2;
    double mux_lane_vec_final2;
    double muy_lane_vec_final2;

    vector<double> mux_lane_vec;
    vector<double> muy_lane_vec;
    vector<double> base_ptx_lane_vec;
    vector<double> base_pty_lane_vec;
    vector<double> angle_lanes;

    int count_lanes_previous;

    vector<double> mux_lane_vec_aggregated;
    vector<double> muy_lane_vec_aggregated;
    vector<double> base_ptx_lane_vec_aggregated;
    vector<double> base_pty_lane_vec_aggregated;

    int count_tracked_lanes2;

    double x1_lane_group1;
    double x1_lane_group2;
    int initial_frame;
    double mux_lane_vec_final1_previous;
    double muy_lane_vec_final1_previous;
    double base_ptx_lane_vec_final1_previous;
    double base_pty_lane_vec_final1_previous;
    double mux_lane_vec_final2_previous;
    double muy_lane_vec_final2_previous;
    double base_ptx_lane_vec_final2_previous;
    double base_pty_lane_vec_final2_previous;
    //int horizontal_tolerance;

    int count_lanes_average_vec;
    int count_lanes_average_vec2;
    vector<double> mux_lane_vec_average;
    vector<double> muy_lane_vec_average;
    vector<double> base_ptx_lane_vec_average;
    vector<double> base_pty_lane_vec_average;
    vector<double> mux_lane_vec_average2;
    vector<double> muy_lane_vec_average2;
    vector<double> base_ptx_lane_vec_average2;
    vector<double> base_pty_lane_vec_average2;

    double muy_lane_vec_final2_speed_permanent;
    double mux_lane_vec_final2_speed_permanent;
    double base_ptx_lane_vec_final2_speed_permanent;
    double base_pty_lane_vec_final2_speed_permanent;

    int override_lane_group2;
    double override_lane_group2_mux;
    double override_lane_group2_muy;
    double override_lane_group2_base_ptx;
    double override_lane_group2_base_pty;
  
    int override_lane_group1;
    double override_lane_group1_mux;
    double override_lane_group1_muy;
    double override_lane_group1_base_ptx;
    double override_lane_group1_base_pty;

    int count_lane_group1_previous;
    int count_lane_group2_previous;
    vector<int> lane_group1_previous;
    vector<int> lane_group2_previous;
    
    vector<double> mux_lane_vec_aggregated_previous;
    vector<double> muy_lane_vec_aggregated_previous;
    vector<double> base_ptx_lane_vec_aggregated_previous;
    vector<double> base_pty_lane_vec_aggregated_previous;

    //vector<int> newvariable;
   
    int count_marks_filtered_grp2;
    int count_marks_filtered_grp1;

    //New variables needed to make speed_estimator_update()
    int first_reading_available_flag;
    int road_nomark;
    int road_nomark_1;
    int white_mark_hit;
    int white_mark_hit_1;
    int count_road_nomark;
    int count_road_nomark_1;
    int capture_frameindex_for_speed;
    int capture_frameindex_for_speed_1;
    int frameindex_for_speed;
    int frameindex_for_speed_1;
    int frameindex_for_speed_previous;
    int frameindex_for_speed_previous_1;
    int count_scanned_lines_reverse_for_speed_previous;
    int count_scanned_lines_for_speed_previous;
    int count_scanned_lines_reverse_for_speed_previous_1;
    int count_scanned_lines_for_speed_previous_1;
    int count_scanned_lines_reverse_for_speed;
    int count_scanned_lines_for_speed;
    int count_scanned_lines_reverse_for_speed_1;
    int count_scanned_lines_for_speed_1;

    double speed_previous;
    double speed_1_previous;
    double speed_hold;
    double speed_1_hold;
    int count_hold;
    int count_1_hold;


    int speed_hold_due_to_delta_official;
    int count_hold_due_to_delta_official;
    int speed_1_hold_due_to_delta_official;
    int count_1_hold_due_to_delta_official;

    double speed_latest;
    double speed_1_latest;
    double limit;

    double time_stamp_for_speed;
    double time_stamp_for_speed_1;
    double time_stamp_for_speed_previous;
    double time_stamp_for_speed_previous_1;




    double speed_official;

    int image_number;    

    int any;   
    


};
