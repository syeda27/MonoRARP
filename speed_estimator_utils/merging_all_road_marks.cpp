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

void Lane_detector::merging_all_road_marks()
{

    //#Merging all found lanes (without repetition) into two lanes
    //#first pick the first lane
    //print("count_tracked_lanes: ",count_tracked_lanes2)
    //cout<<"entered merging"<<endl;    

    vector<int> lane_group1(40,0); //#one group of lanes with positive angle
    vector<int> lane_group2(40,0); //#one group of lanes with negative angle

    //cout<<"this part"<<endl;  
    double mux_lane_vec_acc;
    double muy_lane_vec_acc;
    double base_ptx_lane_vec_acc;
    double base_pty_lane_vec_acc;
    
    count_lane_group1=0;
    count_lane_group2=0;

    mux_lane_vec_final1=0;
    muy_lane_vec_final1=0;
    base_ptx_lane_vec_final1=0;
    base_pty_lane_vec_final1=0;

    mux_lane_vec_final2=0;
    muy_lane_vec_final2=0;
    base_ptx_lane_vec_final2=0;
    base_pty_lane_vec_final2=0;
    
    
    //cout<<"count_tracked_lanes2 "<<count_tracked_lanes2<<endl;
    for (int lanes=0; lanes<count_tracked_lanes2; lanes++)
    {
        if (angle_lanes[lanes]>0)
        {
            //cout<<"count_lane_group1 "<<count_lane_group1<<endl;
            lane_group1[count_lane_group1]=int(lanes); //#storing indexes
            count_lane_group1=count_lane_group1+1;
        }
          
        else
        {
            //cout<<"count_lane_group2 "<<count_lane_group2<<endl;
            lane_group2[count_lane_group2]=int(lanes);
            count_lane_group2=count_lane_group2+1;
        }
    }
            
    //cout<<"this part2"<<endl;

    if (count_lane_group1>=2)
    {
        
         mux_lane_vec_acc=0;
         muy_lane_vec_acc=0;
         base_ptx_lane_vec_acc=0;
         base_pty_lane_vec_acc=0;
        
         for (int k4=0; k4<count_lane_group1; k4++)
         {
                
             mux_lane_vec_acc=mux_lane_vec_acc+mux_lane_vec_aggregated[int(lane_group1[k4])];
             muy_lane_vec_acc=muy_lane_vec_acc+muy_lane_vec_aggregated[int(lane_group1[k4])];
             base_ptx_lane_vec_acc=base_ptx_lane_vec_acc+base_ptx_lane_vec_aggregated[int(lane_group1[k4])];
             base_pty_lane_vec_acc=base_pty_lane_vec_acc+base_pty_lane_vec_aggregated[int(lane_group1[k4])];

         }
             
         mux_lane_vec_final1=mux_lane_vec_acc/count_lane_group1; 
         muy_lane_vec_final1=muy_lane_vec_acc/count_lane_group1;
         base_ptx_lane_vec_final1=base_ptx_lane_vec_acc/count_lane_group1;
         base_pty_lane_vec_final1=base_pty_lane_vec_acc/count_lane_group1;

    }
            
    else
    {

        if (count_lane_group1==1)
        {
          
            mux_lane_vec_final1=mux_lane_vec_aggregated[int(lane_group1[0])];
            muy_lane_vec_final1=muy_lane_vec_aggregated[int(lane_group1[0])];
            base_ptx_lane_vec_final1=base_ptx_lane_vec_aggregated[int(lane_group1[0])];
            base_pty_lane_vec_final1=base_pty_lane_vec_aggregated[int(lane_group1[0])];

        }
    }
        
        
            
    if (count_lane_group2>=2)
    {
        
         mux_lane_vec_acc=0;
         muy_lane_vec_acc=0;
         base_ptx_lane_vec_acc=0;
         base_pty_lane_vec_acc=0;
        
         for (int k4=0 ; k4<count_lane_group2; k4++)
         {
                
             mux_lane_vec_acc=mux_lane_vec_acc+mux_lane_vec_aggregated[int(lane_group2[k4])];
             muy_lane_vec_acc=muy_lane_vec_acc+muy_lane_vec_aggregated[int(lane_group2[k4])];
             base_ptx_lane_vec_acc=base_ptx_lane_vec_acc+base_ptx_lane_vec_aggregated[int(lane_group2[k4])];
             base_pty_lane_vec_acc=base_pty_lane_vec_acc+base_pty_lane_vec_aggregated[int(lane_group2[k4])];

         }         
    
         mux_lane_vec_final2=mux_lane_vec_acc/count_lane_group2; 
         muy_lane_vec_final2=muy_lane_vec_acc/count_lane_group2;
         base_ptx_lane_vec_final2=base_ptx_lane_vec_acc/count_lane_group2;
         base_pty_lane_vec_final2=base_pty_lane_vec_acc/count_lane_group2;
    }
            
    else
    {

        if (count_lane_group2==1)
        {
            
            mux_lane_vec_final2=mux_lane_vec_aggregated[int(lane_group2[0])];
            muy_lane_vec_final2=muy_lane_vec_aggregated[int(lane_group2[0])];
            base_ptx_lane_vec_final2=base_ptx_lane_vec_aggregated[int(lane_group2[0])];
            base_pty_lane_vec_final2=base_pty_lane_vec_aggregated[int(lane_group2[0])];
        }

    }

    return;         

}
