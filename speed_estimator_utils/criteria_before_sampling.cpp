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

int Lane_detector::criteria_before_sampling()
{


    int criteria_flag=0;

    //# b) From all the lines we found inside a scaning region we find the two top line-segments (closest to the top of the frame). 
 
    int first_top=-1;
    int second_top=-1;
    int top;
    //int top_left;
    //int top_right;

                top=10000;
                for (int k1=0; k1<count_angles_per_region; k1++)
                {
                     if ( ry1[k1]<top )
                     {   
                         top=ry1[k1];
                         first_top=k1;
                     }
                }

                top=10000;
                for (int k1=0; k1<count_angles_per_region; k1++)
                {
                     if ((ry1[k1]<top) && (k1!=first_top))
                     {
                         top=ry1[k1];
                         second_top=k1;
                     }
                }

                //# There will be one top segment on the left and another on the right. Out of the two top segments indetify which segment is to the left and which to the right
                if (rx1[first_top]<rx1[second_top])
                {
                    top_left=first_top;   
                    top_right=second_top;
                }
                else
                {
                    top_left=second_top;   
                    top_right=first_top;
                }
               

                //# c) Sampling of 5 pixels around the two top line segments for each horizontal line crossing the two line segments, and detection of signature

                //# criteria that must be complied with to proceed to the sampling:
                //# 1) the top-left line and the top-right line should be aliged vertically so that horizontal lines would intersect both of them (at least for some portion of both)
                //# 2) The difference between the top-left line's angle and the top-right line's angle should be less than 6 degrees
                if ((ry2[top_left]>ry1[top_right]) && (ry2[top_right]>ry1[top_left]) && (abs(angles[top_right]-angles[top_left])<6))
                {
                    criteria_flag=1;
                }

    return criteria_flag;

}
