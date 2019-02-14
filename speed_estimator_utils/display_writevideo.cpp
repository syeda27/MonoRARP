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

void Lane_detector::display(double speed_official,int first_reading_available_flag,int image_number)
{
        string speed_text;
        string file_name_out;

        cv::resize(img6, img7, cv::Size(3840,2880), 0, 0);

        if (first_reading_available_flag!=0)
        {
            speed_text="Speed: "+to_string(int(speed_official))+" miles/hr";
            putText(img7,speed_text.c_str(), Point(250,150), FONT_HERSHEY_SIMPLEX, 4, Scalar(255,255,255),2,LINE_AA);
        }

        namedWindow("frame", WINDOW_NORMAL);
        imshow("frame",img7);
        waitKey(1);

  
        if (image_number<1000)
        {

            file_name_out="/media/juan/6434-3031/GH040008_frames/outvideo/"+to_string(0)+to_string(image_number)+".jpg";
        }

        if (image_number>=1000)
        {

            file_name_out="/media/juan/6434-3031/GH040008_frames/outvideo/"+to_string(image_number)+".jpg";
        }

        //imwrite(file_name_out,img7);

        //initial_frame=1;

}

