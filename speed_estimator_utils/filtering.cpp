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

void Lane_detector::filtering(double horizontal_tolerance)

{

    double angle1;
    double angle2;
    double Lintersection;
    double x1_lane;

    cout<<"initial frame "<<initial_frame<<endl;
    if (count_lane_group1 >= 1)
    {
        
        if ((initial_frame == 1) && (muy_lane_vec_final1_previous !=0 ) && (mux_lane_vec_final1_previous != 0))
        {
            angle1=180*(atan(muy_lane_vec_final1/mux_lane_vec_final1))/3.14159;
            angle2=180*(atan(muy_lane_vec_final1_previous/mux_lane_vec_final1_previous))/3.14159;
            
            if (abs(angle1-angle2)>20)  
            {
                cout<<"abs(angle1-angle2) right "<<abs(angle1-angle2)<<endl;
                cv::resize(img6, img7, cv::Size(3840,2880), 0, 0);

                namedWindow("frame", WINDOW_NORMAL);
                imshow("frame",img7);
                waitKey(1);

                //cin>>any;
                mux_lane_vec_final1=mux_lane_vec_final1_previous;
                muy_lane_vec_final1=muy_lane_vec_final1_previous;
                base_ptx_lane_vec_final1=base_ptx_lane_vec_final1_previous;
                base_pty_lane_vec_final1=base_pty_lane_vec_final1_previous;
            }

        }
        
        
        //#intersecting with top of image        
        Lintersection=-base_pty_lane_vec_final1/muy_lane_vec_final1;
        x1_lane=base_ptx_lane_vec_final1+Lintersection*mux_lane_vec_final1;
        
        
        if ((initial_frame == 1) && (muy_lane_vec_final1_previous !=0 ) && (mux_lane_vec_final1_previous != 0))
        {     
            //cout<<"horizontal right "<<abs(x1_lane_group1-x1_lane)<<endl;
            if (abs(x1_lane_group1-x1_lane) > horizontal_tolerance)
            {
            //#if abs(x1_lane_group1-x1_lane)>80:
                cout<<"horizontal tolerance "<<horizontal_tolerance<<endl;
                cout<<"horizontal right "<<abs(x1_lane_group1-x1_lane)<<endl;
                //cin>>any;
                mux_lane_vec_final1=mux_lane_vec_final1_previous;
                muy_lane_vec_final1=muy_lane_vec_final1_previous;
                base_ptx_lane_vec_final1=base_ptx_lane_vec_final1_previous;
                base_pty_lane_vec_final1=base_pty_lane_vec_final1_previous;
                Lintersection=-base_pty_lane_vec_final1/muy_lane_vec_final1;
                x1_lane=base_ptx_lane_vec_final1+Lintersection*mux_lane_vec_final1;
             }
        
        }

        x1_lane_group1=x1_lane;
        
        
    }    

    else
    {
        if ((muy_lane_vec_final1_previous !=0 ) && (mux_lane_vec_final1_previous != 0))
        {
        
            //#intersecting with top of image        
            Lintersection=-base_pty_lane_vec_final1_previous/muy_lane_vec_final1_previous;
            x1_lane=base_ptx_lane_vec_final1_previous+Lintersection*mux_lane_vec_final1_previous;
        
            mux_lane_vec_final1=mux_lane_vec_final1_previous;
            muy_lane_vec_final1=muy_lane_vec_final1_previous;
            base_ptx_lane_vec_final1=base_ptx_lane_vec_final1_previous;
            base_pty_lane_vec_final1=base_pty_lane_vec_final1_previous;
        
            x1_lane_group1=x1_lane;
        }
        
    }


    if (count_lane_group2 >= 1)
    {

        if ((initial_frame == 1) && (muy_lane_vec_final2_previous !=0 ) && (mux_lane_vec_final2_previous != 0))
        {
            angle1=180*(atan(muy_lane_vec_final2/mux_lane_vec_final2))/3.14159;
            angle2=180*(atan(muy_lane_vec_final2_previous/mux_lane_vec_final2_previous))/3.14159;
            
            if (abs(angle1-angle2)>20)
            {
                cout<<"abs(angle1-angle2) left "<<abs(angle1-angle2)<<endl;
                //cin>>any;
                mux_lane_vec_final2=mux_lane_vec_final2_previous;
                muy_lane_vec_final2=muy_lane_vec_final2_previous;
                base_ptx_lane_vec_final2=base_ptx_lane_vec_final2_previous;
                base_pty_lane_vec_final2=base_pty_lane_vec_final2_previous;
            }
        }
        //#intersecting with top of image        
        
        Lintersection=-base_pty_lane_vec_final2/muy_lane_vec_final2;
        x1_lane=base_ptx_lane_vec_final2+Lintersection*mux_lane_vec_final2;
       
        
        if ((initial_frame == 1) && (muy_lane_vec_final2_previous !=0 ) && (mux_lane_vec_final2_previous != 0))
        {
            //cout<<"horizontal left "<<abs(x1_lane_group1-x1_lane)<<endl;
            if (abs(x1_lane_group2-x1_lane) > horizontal_tolerance)
            {
            //#if abs(x1_lane_group2-x1_lane)>80:
                cout<<"horizontal left "<<abs(x1_lane_group1-x1_lane)<<endl;
                //cin>>any;
                mux_lane_vec_final2=mux_lane_vec_final2_previous;
                muy_lane_vec_final2=muy_lane_vec_final2_previous;
                base_ptx_lane_vec_final2=base_ptx_lane_vec_final2_previous;
                base_pty_lane_vec_final2=base_pty_lane_vec_final2_previous;
                Lintersection=-base_pty_lane_vec_final2/muy_lane_vec_final2;
                x1_lane=base_ptx_lane_vec_final2+Lintersection*mux_lane_vec_final2;
            }
        }
       
        x1_lane_group2=x1_lane;
        
    
    }    

    else
    {
        if ((muy_lane_vec_final2_previous !=0 ) && (mux_lane_vec_final2_previous != 0))
        {
            //#intersecting with top of image        
            Lintersection=-base_pty_lane_vec_final2_previous/muy_lane_vec_final2_previous;
            x1_lane=base_ptx_lane_vec_final2_previous+Lintersection*mux_lane_vec_final2_previous;
   
        
            mux_lane_vec_final2=mux_lane_vec_final2_previous;
            muy_lane_vec_final2=muy_lane_vec_final2_previous;
            base_ptx_lane_vec_final2=base_ptx_lane_vec_final2_previous;
            base_pty_lane_vec_final2=base_pty_lane_vec_final2_previous;
        
            x1_lane_group2=x1_lane;
        }
    }

    return;

}
