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
#include "speed_estimation.h"

using namespace std;
using namespace cv;


   

main()
{

    string file_name;
    double speed_reading;


    Speed_estimator speed_est = Speed_estimator();
    

    for (int image_number=2700; image_number<3600 ;image_number++)
    {

        file_name = "../../GH010034_frames/frames_flipped/"+to_string(image_number)+".jpg";

        speed_est.Speed_estimator_update(file_name, image_number);

        speed_reading = speed_est.get_speed();

        cout<<"speed_reading: "<<speed_reading<<endl;

    }
   
    

}
