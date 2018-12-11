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
#include "../speed_estimator_utils/speed_estimation.h"

using namespace std;
using namespace cv;



int main(int argc, char const *argv[]) {
    string folder_name = "../tests/GH_frames/";
    double speed_reading = 0;

    if(argc > 1) {
      folder_name = argv[1];
    }

    Speed_estimator speed_est = Speed_estimator();
    for (int image_number=2700; image_number<=2800 ;image_number++) {
        string file_name = folder_name+to_string(image_number)+".jpg";
        speed_est.Speed_estimator_update(file_name, image_number);
        speed_reading = speed_est.get_speed();
        cout<<"speed_reading: "<<speed_reading<<endl;
    }
    return 0;
}
