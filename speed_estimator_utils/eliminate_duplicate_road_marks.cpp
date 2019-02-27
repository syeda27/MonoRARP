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

void Lane_detector::eliminate_duplicate_road_marks()
{

    //#we will elminate duplicates on the tracked lines   
    count_tracked_lanes2=0;
    int repeat;

    if (count_lanes!=0)
    {
        
        //#count_tracked_lanes2=0
        for (int lanes=0; lanes<count_lanes; lanes++)
        {
            
            if (lanes>=1)
            {
                repeat=0;
                for (int lanes2=0; lanes2<lanes; lanes2++)
                {
                    if (mux_lane_vec[lanes2]==mux_lane_vec[lanes])
                    {
                        repeat=1; //#we found a repeat, the repeats exist because the scanning region detects more than once
                        break;
                    }
                }

                if (repeat==0)
                {
                    mux_lane_vec_aggregated[count_tracked_lanes2]=mux_lane_vec[lanes];
                    muy_lane_vec_aggregated[count_tracked_lanes2]=muy_lane_vec[lanes];
                    base_ptx_lane_vec_aggregated[count_tracked_lanes2]=base_ptx_lane_vec[lanes];
                    base_pty_lane_vec_aggregated[count_tracked_lanes2]=base_pty_lane_vec[lanes]; 
                    angle_lanes[count_tracked_lanes2]=atan(muy_lane_vec[lanes]/mux_lane_vec[lanes])*180/3.14159;
                    count_tracked_lanes2=count_tracked_lanes2+1;
                }
            }
                    
            else
            {
                mux_lane_vec_aggregated[count_tracked_lanes2]=mux_lane_vec[lanes];
                muy_lane_vec_aggregated[count_tracked_lanes2]=muy_lane_vec[lanes];
                base_ptx_lane_vec_aggregated[count_tracked_lanes2]=base_ptx_lane_vec[lanes];
                base_pty_lane_vec_aggregated[count_tracked_lanes2]=base_pty_lane_vec[lanes]; 
                angle_lanes[count_tracked_lanes2]=atan(muy_lane_vec[lanes]/mux_lane_vec[lanes])*180/3.14159;
                count_tracked_lanes2=count_tracked_lanes2+1;
            }

        }

    }

    return;

}