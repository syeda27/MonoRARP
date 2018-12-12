#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <chrono>
#include "iostream"
#include <vector>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <string>
#include <random>
#include "../speed_estimator_utils/speed_estimation.h"

using namespace std;
using namespace cv;



int main(int argc, char const *argv[]) {
    double speed_reading = 0;
    const double fps = 30.0;
    double curtime = 0.0;
    VideoCapture cap("/scratch/derek/video_captures/FullFOVandHD/video11a.mp4");
    Speed_estimator speed_est2 = Speed_estimator(true);
    // Check if camera opened successfully
    if(!cap.isOpened()){
        cout << "Error opening video stream or file" << endl;
        return -1;
    }
    while(1) {
        Mat frame;
        cap >> frame;
        // If the frame is empty, break immediately
        if (frame.empty()) {
          break;
        }

        curtime += 1.0/fps;
        cout << curtime << endl;
        speed_est2.Speed_estimator_update(frame, curtime);
        speed_reading = speed_est2.get_speed();
        cout<<"speed_reading: "<<speed_reading<<endl;
    }
    return 0;
}
