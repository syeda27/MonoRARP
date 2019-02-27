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

    count_lane_group1_previous=count_lane_group1;
    count_lane_group2_previous=count_lane_group2;

    for (int k4=0;k4<count_lane_group1;k4++)
    {
        lane_group1_previous[k4]=lane_group1[k4];
    }

    for(int k4=0;k4<count_lane_group2;k4++)
    {
        lane_group2_previous[k4]=lane_group2[k4];
    }

    for(int k4=0;k4<count_tracked_lanes2;k4++)
    {
        mux_lane_vec_aggregated_previous[k4]=mux_lane_vec_aggregated[k4];
        muy_lane_vec_aggregated_previous[k4]=muy_lane_vec_aggregated[k4];
        base_ptx_lane_vec_aggregated_previous[k4]=base_ptx_lane_vec_aggregated[k4];
        base_pty_lane_vec_aggregated_previous[k4]=base_pty_lane_vec_aggregated[k4];
    }

}
