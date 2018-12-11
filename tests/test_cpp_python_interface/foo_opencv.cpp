#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>

using namespace cv;

class Foo{
    public:
        void bar(){
            Mat image;
            image = imread("../Hwy101_frames/1085.jpg", CV_LOAD_IMAGE_COLOR);
            if(! image.data )                              // Check for invalid input
            {
                std::cout << "Failed" << std::endl;
                return;
            }

            namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
            imshow( "Display window", image );                   // Show our image inside it.

            waitKey(0);
            std::cout << "Hello" << std::endl;
        }
};

extern "C" {
    Foo* Foo_new(){ return new Foo(); }
    void Foo_bar(Foo* foo){ foo->bar(); }
}
