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

void Lane_detector::brightness_sampling()
{

    int starting_y;
    double veclenght_top_left;
    double veclenght_top_right;
    double mux_top_left;
    double muy_top_left;
    double mux_top_right;
    double muy_top_right;
    //int counter_scanning;

    double L_left;
    double x_border_left;
    double L_right;
    double x_border_right;

    double road1;
    double road2;
    double road3;
    double road4;
    double whitemark;
    int y_scanning;

    

    //#Initializing vectors for collection of road samples
    for (int k=0; k<200; k++)
    {
        road1_vec[k]=0;
        road3_vec[k]=0;    
        road2_vec[k]=0;
        road4_vec[k]=0;
        whitemarkings_vec[k]=0;

    }

    //#Here we are going to sample from the region inside the two line-segments. If it is a Lane then the sampling should provide a white pixel (from the white marking)
    //#we will also sample a pixel to the left of the top-left segment and sample a pixel to the right of the top-right segement. These pixels should belong to the road (they should be nonwhite) 
 
    if (ry1[top_left]>ry1[top_right])
    {
        starting_y=ry1[top_left];
    }
    else
    {
        starting_y=ry1[top_right];
    }
                
         
    //#Calculation of unit vectors ("mu" vectors) in the direction of the top-left line and the top-right line        
    veclenght_top_left=pow((pow((ry2[top_left]-ry1[top_left]),2)+pow((rx2[top_left]-rx1[top_left]),2)),0.5);
    veclenght_top_right=pow((pow((ry2[top_right]-ry1[top_right]),2)+pow((rx2[top_right]-rx1[top_right]),2)),0.5);
    mux_top_left=(rx2[top_left]-rx1[top_left])/veclenght_top_left;
    muy_top_left=(ry2[top_left]-ry1[top_left])/veclenght_top_left;
    mux_top_right=(rx2[top_right]-rx1[top_right])/veclenght_top_right;
    muy_top_right=(ry2[top_right]-ry1[top_right])/veclenght_top_right;
                       


    counter_scanning=0;
    //horizontal_distance_acumulator=0;
                
    //#The sampling of the five pixels is performed by using horizontal lines (with respect to the image subframe). These horizontal lines are represented by y_scanning 
    //#the scanning for the sampling will be performed from top to bottom. At each horizontal scanning we take 5 samples.
    //#Four samples are taken over the horizontal line used for scanning. Two samples are taken at the right side of the top-left line and To samples at the left side of the top-right line
    //#The other sample is taken at the middle of the segment of the horizontal line that is enclosed by the top-left line and the top-right line (sample is white when scanning the Lane mark)
    while ( (starting_y+counter_scanning<ry2[top_right]) && (starting_y+counter_scanning<ry2[top_left]) )
    {

        y_scanning=starting_y+counter_scanning;
          
        //#from analytic geometry we obtain the left-border which is the pixel that belongs to the top-left segment at the y_scanning coordinate 
        //#This is the intersection between the horizontal line and top-left line
        L_left=(y_scanning-ry1[top_left])/muy_top_left;
        x_border_left=rx1[top_left]+ L_left*mux_top_left;  

        //#from analytic geometry we obtain the right-border which is the pixel that belongs to the top-right segment at the y_scanning coordinate
        //#This is the intersection between the horizontal line and the top-right line
        L_right=(y_scanning-ry1[top_right])/muy_top_right;
        //#x_border_right=rx1[top_right]+ L_left*mux_top_right
        x_border_right=rx1[top_right]+ L_right*mux_top_right;

        //#we sample 5 pixels and collect 4 pavement pixels on road1,road2, road3 and road4 and the supposed white pixel on whitemark.
        road1=(int)img2.at<uchar>(int(y_scanning),int(x_border_left-15)); 
        road3=(int)img2.at<uchar>(int(y_scanning),int(x_border_left-30)); 
        whitemark=(int)img2.at<uchar>(int(y_scanning),int((x_border_left+x_border_right)/2));
        road2=(int)img2.at<uchar>(int(y_scanning),int(x_border_right+15)); 
        //road4=(int)img2.at<uchar>(int(y_scanning),int(x_border_left+30)); 
        road4=(int)img2.at<uchar>(int(y_scanning),int(x_border_right+30));
        //cout<<"r3: "<<road3<<" r1: "<<road1<<" w: "<<whitemark<<" r2: "<<road2<<" r4: "<<road4<<endl;
                     
        road1_vec[counter_scanning]=road1;
        road2_vec[counter_scanning]=road2;
        road3_vec[counter_scanning]=road3;
        road4_vec[counter_scanning]=road4;
        whitemarkings_vec[counter_scanning]=whitemark;

        counter_scanning=counter_scanning+1;         

        //#lenght of horizontal segment enclosed by top-left border and top-right border is accumulated as the scanning moves from top to bottom             
        //horizontal_distance_acumulator=horizontal_distance_acumulator+x_border_right-x_border_left 
    }

    return;
}
