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

#include<iostream>
#include<fstream>

using namespace std;
using namespace cv;

void Lane_detector::display(double speed_official,int first_reading_available_flag,int image_number)
{
        string speed_text;
        string file_name_out;

        cv::resize(img4, img7, cv::Size(3840,2880), 0, 0);

        if (first_reading_available_flag!=0)
        {
            speed_text="Speed: "+to_string(int(speed_official))+" miles/hr";
            putText(img7,speed_text.c_str(), Point(250,150), FONT_HERSHEY_SIMPLEX, 4, Scalar(255,255,255),2,LINE_AA);
        }

        namedWindow("frame2", WINDOW_NORMAL);
        imshow("frame2",img7);
        waitKey(1);

}

