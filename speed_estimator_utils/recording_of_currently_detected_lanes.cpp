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

void Lane_detector::recording_of_currently_detected_lanes()
{

    mux_lane_vec_final2_previous=mux_lane_vec_final2;
    muy_lane_vec_final2_previous=muy_lane_vec_final2;
    base_ptx_lane_vec_final2_previous=base_ptx_lane_vec_final2;
    base_pty_lane_vec_final2_previous=base_pty_lane_vec_final2;  
    
    mux_lane_vec_final1_previous=mux_lane_vec_final1;
    muy_lane_vec_final1_previous=muy_lane_vec_final1;
    base_ptx_lane_vec_final1_previous=base_ptx_lane_vec_final1;
    base_pty_lane_vec_final1_previous=base_pty_lane_vec_final1;  

    initial_frame=1;
}
