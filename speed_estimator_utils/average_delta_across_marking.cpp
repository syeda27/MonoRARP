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

void Lane_detector::average_delta()
{
    
    //#Average Delta between the brigthness of each sampled pixel and the mean of the brigthness across the horizontal scanning (for each of the 5 types of sample pixels)
    double delta_road1_accumulator=0;
    double delta_road2_accumulator=0;
    double delta_road3_accumulator=0;
    double delta_road4_accumulator=0;
    double delta_whitemarkings_accumulator=0;

    for (int j1=0; j1<counter_scanning; j1++)
    {
        delta_road1_accumulator=delta_road1_accumulator+abs(road1_vec[j1]-road1_average);
        delta_road2_accumulator=delta_road2_accumulator+abs(road2_vec[j1]-road2_average);

        if ((removed3==0) || (removed3!=j1))
        {
            delta_road3_accumulator=delta_road3_accumulator+abs(road3_vec[j1]-road3_average);
        }

        if ((removed4==0) || (removed4!=j1))
        {    
            delta_road4_accumulator=delta_road4_accumulator+abs(road4_vec[j1]-road4_average);
        }

        delta_whitemarkings_accumulator=delta_whitemarkings_accumulator+abs(whitemarkings_vec[j1]-whitemarkings_average);  
    }                   
                    

    delta_road1_average=delta_road1_accumulator/counter_scanning;
    delta_road2_average=delta_road2_accumulator/counter_scanning;

    if (removed3==0)
    {
        delta_road3_average=delta_road3_accumulator/counter_scanning;
    }
    else
    {
        delta_road3_average=delta_road3_accumulator/(counter_scanning-1);
    }

    if (removed4==0)
    {    
        delta_road4_average=delta_road4_accumulator/counter_scanning;
    }
    else
    {
        delta_road4_average=delta_road4_accumulator/(counter_scanning-1);
    } 
 
    delta_whitemarkings_average=delta_whitemarkings_accumulator/counter_scanning;

    return;

}
