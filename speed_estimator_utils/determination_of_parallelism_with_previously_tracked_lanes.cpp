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

int Lane_detector::determination_of_parallelism()
{
    int aligned_to_tracked_lane=0;
    int proceed;
    double xb;
    double yb;
    double mux_previous_lane;
    double muy_previous_lane;
    double x0;
    double y0;
    double xp;
    double yp;
    double Lproj;
    double distance_to_Lane;
 
                    
    if (whitemarkings_average/road1_average>1)
    {
        if ((count_lane_group1!=0) || (count_lane_group2!=0))
        {
                 
            for (int selection=0; selection<2; selection++) //#we compare against just two previous final lanes
            {
                        
                proceed=0;

                if ((selection==0) && (count_lane_group1!=0))
                {
                    proceed=1;
                    xb=base_ptx_lane_vec_final1;
                    yb=base_pty_lane_vec_final1;
                    mux_previous_lane=mux_lane_vec_final1;
                    muy_previous_lane=muy_lane_vec_final1;
                }
                                    
                if ((selection==1) && (count_lane_group2!=0))
                {
                    proceed=1;
                    xb=base_ptx_lane_vec_final2;
                    yb=base_pty_lane_vec_final2;
                    mux_previous_lane=mux_lane_vec_final2;
                    muy_previous_lane=muy_lane_vec_final2;
                }
                                    
                                                           
                if (proceed==1)
                {    
                    if (abs(angles[top_left]-180*(atan(muy_previous_lane/mux_previous_lane)/3.14159))<6)
                    {
                        
                                                             
                        x0=rx1[top_left];
                        y0=ry1[top_left];
                        Lproj=((x0-xb)*mux_previous_lane+(y0-yb)*muy_previous_lane)/(pow((mux_previous_lane),2)+pow((muy_previous_lane),2));
                                   
                        xp=xb+Lproj*mux_previous_lane;
                        yp=yb+Lproj*muy_previous_lane;
                            
                        distance_to_Lane=pow((pow((x0-xp),2)+pow((y0-yp),2)),0.5);
                                       
                        if (distance_to_Lane<10)
                        {
                            aligned_to_tracked_lane=1;
                        }
                    }

                }


            }  


        }

    }

    return aligned_to_tracked_lane;

}
