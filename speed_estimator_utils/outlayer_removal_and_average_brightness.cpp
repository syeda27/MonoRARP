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

void Lane_detector::outlayer_removal_and_average_brightness_computation()
{


    //#defining accumulators to obatin statistics over brightsness
    double road1_acummulator=0;
    double road2_acummulator=0;
    double road3_acummulator=0;
    double road4_acummulator=0;
    double whitemarkings_acummulator=0;

    int outlayer1;
    int outlayer2;
    int outlayer3;
    int outlayer4;

    double current_average3;
    double current_average4;


    //#### Below we generate some statistics in order to qualify the signature that we are extracting for the segments                                                     #####
    //#### The signature for the white road marking should comply with some criteria applied to the statistics of the samples extracted from the pixels around the marking #####
                

    //#Outlayer Removal and Average Brightness Computation

    //#Outlayer Removal
    //#We take the two samples that are the farthest from the top-left line and the top-right line and we detect and remove one outlayer (pixel value that due to artifact is unsually high)
                
    int removed1=0;
    int removed2=0;
    removed3=0;
    removed4=0;
                
    for (int j1=0; j1<counter_scanning; j1++)
    {
        //#removing only one outlayer (this could be extended to remove more than one outlayer)
        outlayer1=0;
        outlayer2=0;
        outlayer3=0;
        outlayer4=0;
                    
        if (j1>=1)
        {
            if ((abs(road3_vec[j1]-current_average3)>10) && (removed3==0))
            {
                outlayer3=1;
                removed3=j1;
            }
            if ((abs(road4_vec[j1]-current_average4)>10) && (removed4==0))
            {
                outlayer4=1;
                removed4=j1;
            }
        }
                    
        if (outlayer3==0)
        {
            if (removed3==0)
            {
                current_average3=(road3_acummulator+road3_vec[j1])/(j1+1);
            }
            road3_acummulator=road3_acummulator+road3_vec[j1]; 
        } 
                        
                            
        if (outlayer4==0)
        {
            if (removed4==0)
            {
                current_average4=(road4_acummulator+road4_vec[j1])/(j1+1);
            }
            road4_acummulator=road4_acummulator+road4_vec[j1];
        }  
                        
                    
        road1_acummulator=road1_acummulator+road1_vec[j1];
        road2_acummulator=road2_acummulator+road2_vec[j1];
        whitemarkings_acummulator=whitemarkings_acummulator+whitemarkings_vec[j1];

    }

    //#Average Brightness for the 5 samples taken around the two top lines on the scanning region
    road1_average=road1_acummulator/counter_scanning;
    road2_average=road2_acummulator/counter_scanning;

    if (removed3==0)
    {
        road3_average=road3_acummulator/counter_scanning;
    }
    else
    {
        road3_average=road3_acummulator/(counter_scanning-1);
    }
    if (removed4==0)
    {    
        road4_average=road4_acummulator/counter_scanning;
    }
    else
    {
        road4_average=road4_acummulator/(counter_scanning-1);
    }

    whitemarkings_average=whitemarkings_acummulator/counter_scanning;


    return;

}
