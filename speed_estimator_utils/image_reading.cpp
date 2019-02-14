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


void Lane_detector::image_reading(Mat img_color)
{


    int x0;
    int y0;
    int x1; 
    int y1;

    
    //vector<Vec4f> lines_std;

    //#LSD algorithm object creation
    //Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);



    //for (int image_number=1085; image_number<2665 ;image_number++)
    //{

        //file_name = "../Hwy101_frames/"+to_string(image_number)+".jpg";

        //img = imread(file_name.c_str(), IMREAD_GRAYSCALE);
        //img3 = imread(file_name.c_str());

        //img = img_grey;
        img3 = img_color;
        cvtColor(img_color, img, CV_BGR2GRAY);
        
        img(Rect(0,1000,2704-0,1340-1000)).copyTo(img2);
        img3(Rect(0,1000,2704-0,1340-1000)).copyTo(img4);

        H = img.rows;
        W = img.cols;
        h1 = img2.rows;
        w1 = img2.cols;

        img6 = img4.clone();

        count_lanes=0;

        ////#Initialization on every frame
       
        //vector<double> mux_lane_vec(40,0);
        //vector<double> muy_lane_vec(40,0);
        //vector<double> base_ptx_lane_vec(40,0);
        //vector<double> base_pty_lane_vec(40,0);
        //vector<double> angle_lanes(40,0);


        //######## LSD ALGORITHM ########    

        //#LSD line segment detection
        ls->detect(img2, lines_std); //#lines_std holds the lines that have been detected by LSD

        //cout<< lines_std[0][0] << endl;
        //cout<< lines_std.size()<<endl;

        for (int k=0; k<lines_std.size(); k++)
        {
           x0 = int(round(lines_std[k][0]));
           y0 = int(round(lines_std[k][1]));
           x1 = int(round(lines_std[k][2]));
           y1 = int(round(lines_std[k][3]));
           //cv2.line(img2, (x0, y0), (x1,y1), 255, 1, cv2.LINE_AA)  #we are redrawing the lines to emphasize their borders
           line(img2, Point(x0,y0), Point(x1,y1), Scalar(255,255,255), 2, 1);

        }


        //cv::resize(img6, img7, cv::Size(3840,2880), 0, 0);

        //namedWindow("frame", WINDOW_NORMAL);
        //imshow("frame",img7);
        //waitKey(1);

    //}
    return;

}

























