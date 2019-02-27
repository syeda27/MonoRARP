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

int Lane_detector::lane_signature_detection(double brightness_ratio_threshold, int Left_marging_detection, int Right_margin_detection)
{

    
    int signature_detected=0;
    double L_lane;
    double mux_lane;
    double muy_lane;

    double Lintersection;
    double x1_lane;
    double x2_lane;
    
    

    if ((abs(road1_average-road2_average)/road2_average<0.15) && (delta_road1_average<10 and delta_road2_average<10))    
    {
        //#if abs(road3_average-road4_average)/road4_average<0.15 and delta_road3_average<16 and delta_road4_average<16:
        //if ( (abs(road3_average-road4_average)/road4_average<0.25) && (delta_road3_average<16) && (delta_road4_average<16) )
        if ( (abs(road3_average-road4_average)/road4_average<0.15) && (delta_road3_average<16) && (delta_road4_average<16) )
        {

            if (whitemarkings_average/road1_average > brightness_ratio_threshold)
            {
                if (delta_whitemarkings_average<50)
                {
                    //#if rx1[top_left]>1500 and rx1[top_right]<2700:  #focusing the detection around the center of the ego-vehicle

                    if ((rx1[top_left]>Left_marging_detection) && (rx1[top_right]<Right_margin_detection))
                    {
                        //cout<<"Lane Detected"<<endl;
                                        
                        signature_detected=1;
                        L_lane=pow((pow((rx1[top_left]-rx2[top_left]),2)+pow((ry1[top_left]-ry2[top_left]),2)),0.5);
                        mux_lane=(rx1[top_left]-rx2[top_left])/L_lane;
                        muy_lane=(ry1[top_left]-ry2[top_left])/L_lane;
                        //#intersecting with top of image
                        Lintersection=-ry1[top_left]/muy_lane;
                        x1_lane=rx1[top_left]+Lintersection*mux_lane;
                        //#intersection with bottom of image
                        Lintersection=(H-ry1[top_left])/muy_lane;
                        x2_lane=rx1[top_left]+Lintersection*mux_lane;

                        //cout<<top_left<<endl;
                        //cout<<rx1[top_left]<<endl;
                        //cout<<rx2[top_left]<<endl;
                        //cout<<ry1[top_left]<<endl;
                        //cout<<ry2[top_left]<<endl;
                        //cin>>any;
                                     
                                  
                        //line(img4, Point(int(rx1[top_left]), int(ry1[top_left])), Point(int(rx2[top_left]),int(ry2[top_left])), Scalar(0,0,255), 6, 1);
                        //line(img4, Point(int(rx1[top_right]), int(ry1[top_right])), Point(int(rx2[top_right]),int(ry2[top_right])), Scalar(0,255,0), 6, 1);

                        //cv::resize(img6, img7, cv::Size(3840,2880), 0, 0);

                        //namedWindow("frame", WINDOW_NORMAL);
                        //imshow("frame",img7);
                        //waitKey(0);

                                                                               
                                        
                        //if count_lanes_previous!=0:
                                            
                        //    for lanes in range(0,count_lanes_previous):
                                            
                        //        #intersecting with top of image
                        //        Lintersection=-base_pty_lane_vec_previous[lanes]/muy_lane_vec_previous[lanes]
                        //        x1_lane=base_ptx_lane_vec_previous[lanes]+Lintersection*mux_lane_vec_previous[lanes]
                        //        #intersection with bottom of image
                        //        Lintersection=(H-base_pty_lane_vec_previous[lanes])/muy_lane_vec_previous[lanes]
                        //        x2_lane=base_ptx_lane_vec_previous[lanes]+Lintersection*mux_lane_vec_previous[lanes]               
                    
                                        
                        mux_lane_vec[count_lanes]=mux_lane;
                        muy_lane_vec[count_lanes]=muy_lane;
                        base_ptx_lane_vec[count_lanes]=rx1[top_left];
                        base_pty_lane_vec[count_lanes]=ry1[top_left];
                         
                        count_lanes=count_lanes+1;
                    }
                }
            }
        }
    }

    return signature_detected;

}
