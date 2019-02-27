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

main()
{
    string file_name;
    Mat image_color_from_file;
    double speed_reading;
    double time=0;

    Lane_detector lane_detect = Lane_detector();

    for (int image_number1=3200; image_number1<3500 ;image_number1++)
    {

        file_name = "/media/juan/3036-3031/GH090041_frames/frames_flipped/"+to_string(image_number1)+".jpeg.jpg";

        image_color_from_file = imread(file_name.c_str());

        time = image_number1;

        lane_detect.speed_estimator_update(image_color_from_file, time);

        speed_reading = lane_detect.get_speed();

        cout<<"speed_reading: "<<speed_reading<<endl;
    
    }

}
