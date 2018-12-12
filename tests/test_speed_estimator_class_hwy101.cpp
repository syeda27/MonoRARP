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
    string folder_name = "../tests/GH_frames/";
    double speed_reading = 0;

    if(argc > 1) {
      folder_name = argv[1];
    }

    Speed_estimator speed_est = Speed_estimator(true);
    /*
    for (int image_number=2700; image_number<=2800 ;image_number++) {
        string file_name = folder_name+to_string(image_number)+".jpg";
        speed_est.Speed_estimator_update(file_name, image_number);
        speed_reading = speed_est.get_speed();
        cout<<"speed_reading: "<<speed_reading<<endl;
    }
    */
    // First, we do not force fps, just to make sure it still works.
    /*
    clock_t start = clock();
    for (int image_number=2700; image_number<=2750 ;image_number++) {
        double cur_time = double(clock() - start) / CLOCKS_PER_SEC;
        if (image_number == 2700) {
            cur_time = 0.0;
        }
        cout << cur_time << endl;
        string file_name = folder_name+to_string(image_number)+".jpg";
        Mat img3 = imread(file_name.c_str());
        speed_est.Speed_estimator_update(img3, cur_time);
        speed_reading = speed_est.get_speed();
        cout<<"speed_reading: "<<speed_reading<<endl;
    }
    cout << "took " <<  double(clock() - start) / CLOCKS_PER_SEC << " seconds." << endl;
    */
    // Next, we want to check that if we force fps, since the videos were recorded
    // at a different time than we are running this, let's make sure it gets the
    // correct-ish value
    const double fps = 30.0;
    double curtime = 0.0;
    Speed_estimator speed_est2 = Speed_estimator();
    for (int image_number=2700; image_number<=2800 ;image_number++) {
        curtime += 1.0/fps;
        cout << curtime << endl;
        string file_name = folder_name+to_string(image_number)+".jpg";
        Mat img3 = imread(file_name.c_str());
        cout << img3.rows << " " << img3.cols << endl;
        speed_est2.Speed_estimator_update(img3, curtime);
        speed_reading = speed_est2.get_speed();
        cout<<"speed_reading: "<<speed_reading<<endl;
    }
    assert(int(speed_reading) == 37);
    return 0;
}
