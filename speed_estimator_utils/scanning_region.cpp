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

int Lane_detector::scan_region(int xregion,int yregion,double region_lenght,double region_width)
{

    int x0;
    int y0;
    int x1; 
    int y1;
    double pt1_x;
    double pt1_y;
    double pt2_x;
    double pt2_y;
    double angle;

    //rectangle(img6, Point(xregion-int(region_width/2),yregion-int(region_lenght/2)), Point(xregion+int(region_width/2),yregion+int(region_lenght/2)), Scalar(255,255,255), 2, 1);

    //cv::resize(img6, img7, cv::Size(3840,2880), 0, 0);

    //namedWindow("frame", WINDOW_NORMAL);
    //imshow("frame",img7);
    //waitKey(1);
    
    //#Initializing vectors to be used for collection of the lines inside the region
    for (int k=0; k<100; k++)
    {
        rx1[k]=0;
        rx2[k]=0;
        ry1[k]=0;
        ry2[k]=0;
        angles[k]=0;
    }

    count_angles_per_region=0;


    //#For all the lines detected in the image subframe we collect each line that falls inside the scanning region and we analize it.
    for (int k=0; k<lines_std.size(); k++)
        {
           x0 = int(round(lines_std[k][0]));
           y0 = int(round(lines_std[k][1]));
           x1 = int(round(lines_std[k][2]));
           y1 = int(round(lines_std[k][3]));

           if ( (x0>xregion-int(region_width/2)) && (x0<xregion+int(region_width/2)) && (y0>yregion-int(region_lenght/2)) && (y0<yregion+int(region_lenght/2)) && (x1>xregion-int(region_width/2)) &&  (x1<xregion+int(region_width/2)) && (y1>yregion-int(region_lenght/2)) && (y1<yregion+int(region_lenght/2)) )
           {
               

               //#we establish a convention for each line where pt1 is the line's end-point that is in top. Thus pt2 is below pt1.
               if (y1>y0)
               {
                  pt1_y=y0;
                  pt2_y=y1;
                  pt1_x=x0;
                  pt2_x=x1;
               }
               else
               {
                  pt1_y=y1;
                  pt2_y=y0;
                  pt1_x=x1;
                  pt2_x=x0;
               }


               //#we generate arrays that allow to correlate each line's end-point coordinates with the line's angle. The angle will be used as filtering criteria below.
               if (pt2_x!=pt1_x)  //#we rearely have pt1_x==pt2_x due to perspective. The exception will be a lane changing maneuver (in this case for a moment the lines corresponding to white markings become vertical. In that case we will have a temporal exception that nontheless should not impact performance)
               {
                   

                   angle=180*(atan((pt2_y-pt1_y)/(pt2_x-pt1_x)))/3.14159;
                   //cout<<"angle: "<<angle<<endl;
                   //cin>>any;

                   if ((abs(angle)>20) && (abs(angle)<70)) 
                   {
                       angles[count_angles_per_region]=angle;

                       rx1[count_angles_per_region]=pt1_x;
                       rx2[count_angles_per_region]=pt2_x;
                       ry1[count_angles_per_region]=pt1_y;
                       ry2[count_angles_per_region]=pt2_y;    

                       count_angles_per_region=count_angles_per_region+1;
                   }
               }
           }


        }

    return count_angles_per_region;
}

  
Lane_detector::~Lane_detector()
{
}
