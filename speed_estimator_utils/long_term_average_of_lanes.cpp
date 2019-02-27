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

void Lane_detector::long_term_average_of_lanes(int average_window, double &muy_lane_ave2, double &mux_lane_ave2, double &base_ptx_ave2, double &base_pty_ave2, double &muy_lane_ave, double &mux_lane_ave, double &base_ptx_ave, double &base_pty_ave, int &count_lanes_average_vec2_out,int &count_lanes_average_vec2_out2)
{

    double Lintersection;
    double x1_lane;
    double x2_lane;
    //double mux_lane_ave;
    //double muy_lane_ave;
    //double base_ptx_ave;
    //double base_pty_ave;
    //double mux_lane_ave2;
    //double muy_lane_ave2;
    //double base_ptx_ave2;
    //double base_pty_ave2;

    if ((count_lane_group1>0) && (count_marks_filtered_grp1>0))  
    {
        mux_lane_vec_average[count_lanes_average_vec]=mux_lane_vec_final1;
        muy_lane_vec_average[count_lanes_average_vec]=muy_lane_vec_final1;
        base_ptx_lane_vec_average[count_lanes_average_vec]=base_ptx_lane_vec_final1;
        base_pty_lane_vec_average[count_lanes_average_vec]=base_pty_lane_vec_final1;
        
        //cout<<"override_lane_group1 "<<override_lane_group1<<endl;
        //cout<<"mux_lane_vec_final1 "<<mux_lane_vec_final1<<endl;
        //cout<<"muy_lane_vec_final1 "<<muy_lane_vec_final1<<endl;
        //cout<<"base_ptx_lane_vec_final1 "<<base_ptx_lane_vec_final1<<endl;
        //cout<<"base_pty_lane_vec_final1 "<<base_pty_lane_vec_final1<<endl;
    
        count_lanes_average_vec=count_lanes_average_vec+1;
        //cout<<"count_lanes_average_vec "<<count_lanes_average_vec<<endl;
        //cin>>any;
    }

    if ((count_lane_group1>0) && (override_lane_group1>0))
    {
        if (count_lanes_average_vec<average_window)
        {
            for (int k8=0; k8<average_window; k8++)
            {   
                mux_lane_vec_average[k8]=override_lane_group1_mux;
                muy_lane_vec_average[k8]=override_lane_group1_muy;
                base_ptx_lane_vec_average[k8]=override_lane_group1_base_ptx;
                base_pty_lane_vec_average[k8]=override_lane_group1_base_pty;
            }
        }

        count_lanes_average_vec=count_lanes_average_vec+1;
    }
 
    

    if ((count_lane_group2>0) && (count_marks_filtered_grp2>0)) 
    {
        mux_lane_vec_average2[count_lanes_average_vec2]=mux_lane_vec_final2;
        muy_lane_vec_average2[count_lanes_average_vec2]=muy_lane_vec_final2;
        base_ptx_lane_vec_average2[count_lanes_average_vec2]=base_ptx_lane_vec_final2;
        base_pty_lane_vec_average2[count_lanes_average_vec2]=base_pty_lane_vec_final2;
        
        count_lanes_average_vec2=count_lanes_average_vec2+1;
    }  


    if ((count_lane_group2>0) && (override_lane_group2>0))
    {
        if (count_lanes_average_vec<average_window)
        {
            for (int k8=0; k8<average_window; k8++)
            {   
                mux_lane_vec_average2[k8]=override_lane_group2_mux;
                muy_lane_vec_average2[k8]=override_lane_group2_muy;
                base_ptx_lane_vec_average2[k8]=override_lane_group2_base_ptx;
                base_pty_lane_vec_average2[k8]=override_lane_group2_base_pty;
            }
        }

        count_lanes_average_vec2=count_lanes_average_vec2+1;
    }
    

    
    
    //#if count_lanes_average_vec>=6:
    //#if count_lanes_average_vec>=2: 
    if (count_lanes_average_vec>=average_window)
    {
        double mux_lane_acc=0;
        double muy_lane_acc=0;
        double base_ptx_acc=0;
        double base_pty_acc=0;
        //#for k8 in range(0,6): 
        //#for k8 in range(0,2): 

        for (int k8=0; k8<average_window; k8++)
        {   
            mux_lane_acc=mux_lane_acc+mux_lane_vec_average[count_lanes_average_vec-1-k8];
            muy_lane_acc=muy_lane_acc+muy_lane_vec_average[count_lanes_average_vec-1-k8];
            base_ptx_acc=base_ptx_acc+base_ptx_lane_vec_average[count_lanes_average_vec-1-k8];
            base_pty_acc=base_pty_acc+base_pty_lane_vec_average[count_lanes_average_vec-1-k8];
        }
        //#mux_lane_ave=mux_lane_acc/6
        //#muy_lane_ave=muy_lane_acc/6
        //#base_ptx_ave=base_ptx_acc/6
        //#base_pty_ave=base_pty_acc/6
        //#mux_lane_ave=mux_lane_acc/2
        //#muy_lane_ave=muy_lane_acc/2
        //#base_ptx_ave=base_ptx_acc/2
        //#base_pty_ave=base_pty_acc/2
        mux_lane_ave=mux_lane_acc/average_window;
        muy_lane_ave=muy_lane_acc/average_window;
        base_ptx_ave=base_ptx_acc/average_window;
        base_pty_ave=base_pty_acc/average_window;

        if (override_lane_group1==1)
        {
            mux_lane_ave=override_lane_group1_mux;
            muy_lane_ave=override_lane_group1_muy;
            base_ptx_ave=override_lane_group1_base_ptx;
            base_pty_ave=override_lane_group1_base_pty;


            for (int k8=0; k8<average_window; k8++)
            {   
                mux_lane_vec_average[count_lanes_average_vec-1-k8]=override_lane_group1_mux;
                muy_lane_vec_average[count_lanes_average_vec-1-k8]=override_lane_group1_muy;
                base_ptx_lane_vec_average[count_lanes_average_vec-1-k8]=override_lane_group1_base_ptx;
                base_pty_lane_vec_average[count_lanes_average_vec-1-k8]=override_lane_group1_base_pty;
            }
        }

        //cout<<"prob mux_lane_ave "<<mux_lane_ave<<endl;
        //cout<<"prob muy_lane_ave "<<muy_lane_ave<<endl;
        //cout<<"prob base_ptx_ave "<<base_ptx_ave<<endl;
        //cout<<"prob base_pty_ave "<<base_pty_ave<<endl;

        
        //#intersecting with top of image        
        Lintersection=-base_pty_ave/muy_lane_ave;
        x1_lane=base_ptx_ave+Lintersection*mux_lane_ave;
        //#intersection with bottom of image
        Lintersection=(H-base_pty_ave)/muy_lane_ave;
        //#Lintersection=(h1-base_pty_ave)/muy_lane_ave
        x2_lane=base_ptx_ave+Lintersection*mux_lane_ave;
        
        line(img4, Point(int(base_ptx_ave), int(base_pty_ave)), Point(int(x1_lane),int(0)), Scalar(255,255,255), 1, 1);
        line(img4, Point(int(base_ptx_ave), int(base_pty_ave)), Point(int(x2_lane),int(H)), Scalar(255,255,255), 1, 1);   

        //if count_lanes_average_vec==98:

        //   muy_lane_vec_final1_speed_permanent=muy_lane_ave
        //   mux_lane_vec_final1_speed_permanent=mux_lane_ave
        //   base_ptx_lane_vec_final1_speed_permanent=base_ptx_ave
        //   base_pty_lane_vec_final1_speed_permanent=base_pty_ave

        //   print("muy_lane_vec_final1_speed_permanent: ",muy_lane_vec_final1_speed_permanent)
        //   print("mux_lane_vec_final1_speed_permanent: ",mux_lane_vec_final1_speed_permanent)
        //   print("base_ptx_lane_vec_final1_speed_permanent: ",base_ptx_lane_vec_final1_speed_permanent)
        //   print("base_pty_lane_vec_final1_speed_permanent: ",base_pty_lane_vec_final1_speed_permanent)
        //   input("permanent")

    }

    //#if count_lanes_average_vec2>=6:
    //#if count_lanes_average_vec2>=2:

    if (count_lanes_average_vec2>=average_window)
    {
        double mux_lane_acc=0;
        double muy_lane_acc=0;
        double base_ptx_acc=0;
        double base_pty_acc=0;

        //#for k8 in range(0,2):  
        for (int k8=0; k8<average_window; k8++)
        {    
            mux_lane_acc=mux_lane_acc+mux_lane_vec_average2[count_lanes_average_vec2-1-k8];
            muy_lane_acc=muy_lane_acc+muy_lane_vec_average2[count_lanes_average_vec2-1-k8];
            base_ptx_acc=base_ptx_acc+base_ptx_lane_vec_average2[count_lanes_average_vec2-1-k8];
            base_pty_acc=base_pty_acc+base_pty_lane_vec_average2[count_lanes_average_vec2-1-k8];
        }

        //#mux_lane_ave2=mux_lane_acc/6
        //#muy_lane_ave2=muy_lane_acc/6
        //#base_ptx_ave2=base_ptx_acc/6
        //#base_pty_ave2=base_pty_acc/6
        //#mux_lane_ave2=mux_lane_acc/2
        //#muy_lane_ave2=muy_lane_acc/2
        //#base_ptx_ave2=base_ptx_acc/2
        //#base_pty_ave2=base_pty_acc/2

        mux_lane_ave2=mux_lane_acc/average_window;
        muy_lane_ave2=muy_lane_acc/average_window;
        base_ptx_ave2=base_ptx_acc/average_window;
        base_pty_ave2=base_pty_acc/average_window;

        if (override_lane_group2==1)
        {
            mux_lane_ave2=override_lane_group2_mux;
            muy_lane_ave2=override_lane_group2_muy;
            base_ptx_ave2=override_lane_group2_base_ptx;
            base_pty_ave2=override_lane_group2_base_pty;


            for (int k8=0; k8<average_window; k8++)
            {   
                mux_lane_vec_average2[count_lanes_average_vec2-1-k8]=override_lane_group2_mux;
                muy_lane_vec_average2[count_lanes_average_vec2-1-k8]=override_lane_group2_muy;
                base_ptx_lane_vec_average2[count_lanes_average_vec2-1-k8]=override_lane_group2_base_ptx;
                base_pty_lane_vec_average2[count_lanes_average_vec2-1-k8]=override_lane_group2_base_pty;
            }
        }

        
        //#intersecting with top of image        
        Lintersection=-base_pty_ave2/muy_lane_ave2;
        x1_lane=base_ptx_ave2+Lintersection*mux_lane_ave2;
        //#intersection with bottom of image
        Lintersection=(H-base_pty_ave2)/muy_lane_ave2;
        //#Lintersection=(h1-base_pty_ave2)/muy_lane_ave2
        x2_lane=base_ptx_ave2+Lintersection*mux_lane_ave2;
    
        line(img4, Point(int(base_ptx_ave2), int(base_pty_ave2)), Point(int(x1_lane),int(0)), Scalar(255,255,255), 1, 1);
        line(img4, Point(int(base_ptx_ave2), int(base_pty_ave2)), Point(int(x2_lane),int(H)), Scalar(255,255,255), 1, 1);
    }  

    count_lanes_average_vec2_out2=count_lanes_average_vec2;
    count_lanes_average_vec2_out=count_lanes_average_vec;

    return;
}
