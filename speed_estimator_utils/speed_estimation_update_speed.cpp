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

void Lane_detector::speed_estimator_update(Mat image_color, double time)
{
   

    //#inital and end points for the scanning in the x-direction within the image subframe, and step of the scanning
    int scan_x_ini=900;
    int scan_x_end=1800;
    int scan_x_step=50;//50;

    //#inital and end points for the scanning in the y-direction within the image subframe, and step of the scanning
    int scan_y_ini=130;
    int scan_y_end=220;
    int scan_y_step=20;//20;

    //#size of the rectangular window used for the scanning
    double scanning_window_width=220;//200;//220;//200;
    double scanning_window_lenght=220;//240;//220;

    //#threshold for brightness ratio between white road mark and pavement
    double brightness_ratio_threshold = 1.5;

    //#tolerance for Lane deviation from previously detected Lanes
    double horizontal_tolerance = 30;//30;//70;//180;

    //#Restrict the area for lane detection. Left marging and right marging are the limits of this area 
    int Left_marging_detection = 900;
    int Right_margin_detection = 1800;

    //#Long term average window size
    int average_window = 8;



    string file_name;
    string speed_text;

    int count_angles_per_region;

    int criteria_flag=0;
    int aligned_to_tracked_lane_flag=0;
    int signature_detected_flag=0;

    //int h1;
    //int w1;

    //int count_lanes=0; 

    //int x0;
    //int y0;
    //int x1;
    //int y1;

    double speed=0;
    //int road_nomark=0;
    //int capture_frameindex_for_speed=0;
    //int frameindex_for_speed=0;
    //int white_mark_hit=0;
    int speed_read_flag=0;
    //int count_scanned_lines_reverse_for_speed=0;
    //int count_scanned_lines_for_speed=0;
    int offset=0;
    int offset_at_end_of_mark=0;
    int offset_at_middle_of_mark=0;
        
    //int frameindex_for_speed_previous=0;
    //int image_number=0;
    //int count_scanned_lines_reverse_for_speed_previous=0;
    //int count_scanned_lines_for_speed_previous=0;
    double offset_adjustment=0;

    double speed_1=0;
    //int road_nomark_1=0;
    //int capture_frameindex_for_speed_1=0;
    //int frameindex_for_speed_1=0;
    //int white_mark_hit_1=0;
    int speed_read_flag_1=0;
    //int count_scanned_lines_reverse_for_speed_1=0;
    //int count_scanned_lines_for_speed_1=0;
    int offset_1=0;
    int offset_at_end_of_mark_1=0;
    int offset_at_middle_of_mark_1=0;
        
    //int frameindex_for_speed_previous_1=0;
    //int image_number=0;
    //int count_scanned_lines_reverse_for_speed_previous_1=0;
    //int count_scanned_lines_for_speed_previous_1=0;
    double offset_adjustment_1=0;

    //double speed_official;
    //int first_reading_available_flag=0;
    //int count_road_nomark=0;
    //int count_road_nomark_1=0;

    //double speed_previous=-1;
    //double speed_1_previous=-1;
    //double speed_hold=0;
    //double speed_1_hold=0;
    //int count_hold=0;
    //int count_1_hold=0;


    //int speed_hold_due_to_delta_official=0;
    //int count_hold_due_to_delta_official=0;
    //int speed_1_hold_due_to_delta_official=0;
    //int count_1_hold_due_to_delta_official=0;

    //double speed_latest;
    //double speed_1_latest;
    //double limit;

    
    int any;

    Lane_detector lane_detect = Lane_detector();

    ////vector<Vec4f> lines_std;

    ////#LSD algorithm object creation
    //Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);

    //FIRST THING IS TO INCREASE image_number COUNTER
    image_number=image_number+1;


    
        Lane_detector::image_reading(image_color);

        for (int xregion=scan_x_ini; xregion<=scan_x_end; xregion += scan_x_step)
        {

            for (int yregion=scan_y_ini; yregion<=scan_y_end; yregion += scan_y_step)
            {

  
                count_angles_per_region = Lane_detector::scan_region(xregion,yregion,scanning_window_lenght,scanning_window_width);

                if (count_angles_per_region>=2)
                {

                    criteria_flag = Lane_detector::criteria_before_sampling();
                    //cout<<"criteria: "<<criteria_flag<<endl;

                    if (criteria_flag == 1)
                    {
                        
                        Lane_detector::brightness_sampling();

                        Lane_detector::outlayer_removal_and_average_brightness_computation();

                        Lane_detector::average_delta();

                        //aligned_to_tracked_lane_flag = lane_detect.determination_of_parallelism();

                        signature_detected_flag = Lane_detector::lane_signature_detection( brightness_ratio_threshold, Left_marging_detection, Right_margin_detection );

                    }

                }

            }
        }
        //cout<<"blk 2"<<endl;
        Lane_detector::eliminate_duplicate_road_marks();
        //cout<<"blk 3"<<endl;
        Lane_detector::merging_all_road_marks();
        //cout<<"blk 4"<<endl;
        Lane_detector::filtering(horizontal_tolerance);
        //cout<<"blk 5"<<endl;
        Lane_detector::recording_of_currently_detected_lanes();
        //cout<<"blk 6"<<endl;
        double muy_lane_vec_final2_speed_permanent;
        double mux_lane_vec_final2_speed_permanent;
        double base_ptx_lane_vec_final2_speed_permanent;
        double base_pty_lane_vec_final2_speed_permanent;
        double muy_lane_vec_final1_speed_permanent;
        double mux_lane_vec_final1_speed_permanent;
        double base_ptx_lane_vec_final1_speed_permanent;
        double base_pty_lane_vec_final1_speed_permanent;
        int count_lanes_average_vec2_out;
        int count_lanes_average_vec2_out2;

        Lane_detector::long_term_average_of_lanes(average_window,muy_lane_vec_final2_speed_permanent,mux_lane_vec_final2_speed_permanent,base_ptx_lane_vec_final2_speed_permanent,base_pty_lane_vec_final2_speed_permanent,muy_lane_vec_final1_speed_permanent,mux_lane_vec_final1_speed_permanent,base_ptx_lane_vec_final1_speed_permanent,base_pty_lane_vec_final1_speed_permanent,count_lanes_average_vec2_out,count_lanes_average_vec2_out2);
        //cout<<"blk 7"<<endl;
        int range_adjustment_left=70;//50;
        int range_adjustment_right=0;
        int range_adjustment_right_1=50;
        int range_adjustment_left_1=0;


        //#Speed on left track (group2)    
        speed_read_flag=0; 

        if (count_lanes_average_vec2_out2>=average_window)
        {
            Lane_detector::absolute_speed_estimation(speed, road_nomark, capture_frameindex_for_speed, frameindex_for_speed, white_mark_hit, speed_read_flag, count_scanned_lines_reverse_for_speed, count_scanned_lines_for_speed, offset, offset_at_end_of_mark, offset_at_middle_of_mark, muy_lane_vec_final2_speed_permanent, mux_lane_vec_final2_speed_permanent, base_ptx_lane_vec_final2_speed_permanent, base_pty_lane_vec_final2_speed_permanent, frameindex_for_speed_previous, image_number, count_scanned_lines_reverse_for_speed_previous, count_scanned_lines_for_speed_previous, offset_adjustment, range_adjustment_left, range_adjustment_right, time, time_stamp_for_speed, time_stamp_for_speed_previous);

            if (speed_read_flag==1)
            {
                speed_latest=speed;
                if (speed_previous!=-1)
                {
                    if (speed_previous>45)
                    {limit = 0.35;}
                    else
                    {limit = 0.45;}

                    if ( (abs(speed-speed_previous)/speed_previous>limit) && (abs(speed-speed_1_latest)/speed>=0.2) )
                    {
                        speed_hold=1;
                        count_hold=count_hold+1;
                        cout<<"speed left holded: "<<speed<<endl;
                    }
                    else
                    {

                      if ( ((abs(speed-speed_official)/speed_official>0.7) && (speed_hold==0)) && (abs(speed-speed_1_latest)/speed>=0.2) )
                      {
                        count_hold_due_to_delta_official=count_hold_due_to_delta_official+1;
                        speed_hold_due_to_delta_official=1;
                        cout<<"speed left delta holded: "<<speed<<endl;
                      }                       
                      else
                      {
                        speed_previous=speed;
                        speed_official=speed;
                        cout<<"speed left: "<<speed<<endl;
                        cout<<"speed_1_hold_due_to_delta_official: "<<speed_1_hold_due_to_delta_official<<endl;
                        cout<<"speed_1: "<<speed_1_latest<<endl;
                        speed_hold=0;
                        count_hold=0;
                        if ( (speed_1_hold==1) || (speed_1_hold_due_to_delta_official==1) )
                        {
                            if (abs(speed-speed_1_latest)/speed<0.2)
                            {
                                speed_1_hold=0;
                                count_1_hold=0;
                                speed_1_previous=speed_1_latest;
                                cout<<"right released by left"<<endl;
                                speed_1_hold_due_to_delta_official=0;
                                count_1_hold_due_to_delta_official=0;
                                cout<<"right delta released by left"<<endl;
                            }
                        }
                      }

                    }
                }
                else
                {
                    speed_official=speed;
                    speed_previous=speed;
                    first_reading_available_flag=1;
                }
                //cout<<"speed_read_flag"<<speed_read_flag<<endl;
                //cout<<"frameindex_for_speed "<<frameindex_for_speed<<endl;
                //cout<<"frameindex_for_speed_previous: "<<frameindex_for_speed_previous<<endl;
                //cin>>any;
                if (count_hold>=45)
                {
                    speed_official=speed;
                    speed_previous=speed;  
                    speed_hold=0;
                    count_hold=0; 
                    cout<<"count hold left over 30 released"<<endl;
                }
                if (count_hold_due_to_delta_official>=45)
                {

                    speed_official=speed;
                    speed_previous=speed;
                    speed_hold_due_to_delta_official=0;
                    count_hold_due_to_delta_official=0;
                    cout<<"count hold left delta over 30 released"<<endl;
                }

            }

            if ((road_nomark==1) && (white_mark_hit==1))
            {
                count_road_nomark=count_road_nomark+1;
            }

            //# Detecting ending of white road mark (the marker is over the pavement)
            if (count_road_nomark==5)
            {
                white_mark_hit=0;
                count_road_nomark=0;
                capture_frameindex_for_speed=0;
                frameindex_for_speed_previous=frameindex_for_speed;
                time_stamp_for_speed_previous=time_stamp_for_speed;
                count_scanned_lines_reverse_for_speed_previous=count_scanned_lines_reverse_for_speed;
                count_scanned_lines_for_speed_previous=count_scanned_lines_for_speed;
            }

        }

        speed_read_flag_1=0;

        if (count_lanes_average_vec2_out>=average_window)
        {
            Lane_detector::absolute_speed_estimation( speed_1, road_nomark_1, capture_frameindex_for_speed_1, frameindex_for_speed_1, white_mark_hit_1, speed_read_flag_1, count_scanned_lines_reverse_for_speed_1, count_scanned_lines_for_speed_1, offset_1, offset_at_end_of_mark_1, offset_at_middle_of_mark_1, muy_lane_vec_final1_speed_permanent, mux_lane_vec_final1_speed_permanent, base_ptx_lane_vec_final1_speed_permanent, base_pty_lane_vec_final1_speed_permanent, frameindex_for_speed_previous_1, image_number, count_scanned_lines_reverse_for_speed_previous_1, count_scanned_lines_for_speed_previous_1, offset_adjustment_1, range_adjustment_left_1, range_adjustment_right_1, time, time_stamp_for_speed_1, time_stamp_for_speed_previous_1);
        }

        if (speed_read_flag_1==1)
        {
            speed_1_latest=speed_1;
            //speed filtering
            if (speed_1_previous!=-1)
            {

                if (speed_1_previous>45)
                {limit = 0.35;}
                else
                {limit = 0.45;}

                if ( (abs(speed_1-speed_1_previous)/speed_1_previous>limit) && (abs(speed_latest-speed_1)/speed_1>=0.2) )
                {
                    speed_1_hold=1;
                    cout<<"speed right holded: "<<speed_1<<endl;
                   
                }
                else
                {
                  if ( ((abs(speed_1-speed_official)/speed_official>0.7) && (speed_1_hold==0)) && (abs(speed_latest-speed_1)/speed_1>=0.2) )
                  {
                    count_1_hold_due_to_delta_official=count_1_hold_due_to_delta_official+1;
                    speed_1_hold_due_to_delta_official=1;
                    cout<<"speed right delta holded: "<<speed_1<<endl;
                  }   
                  else
                  {
                    speed_1_previous=speed_1;
                    speed_official=speed_1;
                    cout<<"speed right: "<<speed_1<<endl;
                    cout<<"speed_hold_due_to_delta_official: "<<speed_hold_due_to_delta_official<<endl;
                    cout<<"speed: "<<speed_latest<<endl;
                    speed_1_hold=0;
                    count_1_hold=0;
                    if ( (speed_hold==1) || (speed_hold_due_to_delta_official==1) )
                    {
                        if (abs(speed_latest-speed_1)/speed_1<0.2)
                        {
                            speed_hold=0;
                            count_hold=0;
                            speed_previous=speed_latest;
                            cout<<"left released by right"<<endl;
                            speed_hold_due_to_delta_official=0;
                            count_hold_due_to_delta_official=0;
                            cout<<"left delta released by right"<<endl;
                        }
                    }
                  }

                }
            }
            else
            {
                speed_official=speed_1;
                speed_1_previous=speed_1;
                first_reading_available_flag=1;
            }

            if (count_1_hold>=45)
            {
                speed_official=speed_1;
                speed_1_previous=speed_1;  
                speed_1_hold=0;
                count_1_hold=0; 
                cout<<"count hold right over 30 released"<<endl;
            }
            if (count_1_hold_due_to_delta_official>=45)
            {
                speed_official=speed_1;
                speed_1_previous=speed_1;
                speed_1_hold_due_to_delta_official=0;
                count_1_hold_due_to_delta_official=0;
                cout<<"count hold right delta over 30 released"<<endl;
            }
            
            //cout<<"speed_read_flag_1"<<speed_read_flag_1<<endl;
            //cout<<"speed_1: "<<speed_1<<endl;
        }


        if ((road_nomark_1==1) && (white_mark_hit_1==1))
        {
            count_road_nomark_1=count_road_nomark_1+1;
        }

        //# Detecting ending of white road mark (the marker is over the pavement)
        if (count_road_nomark_1==5)
        {
            white_mark_hit_1=0;
            count_road_nomark_1=0;
            capture_frameindex_for_speed_1=0;
            frameindex_for_speed_previous_1=frameindex_for_speed_1;
            time_stamp_for_speed_previous_1=time_stamp_for_speed_1;
            count_scanned_lines_reverse_for_speed_previous_1=count_scanned_lines_reverse_for_speed_1;
            count_scanned_lines_for_speed_previous_1=count_scanned_lines_for_speed_1;
        }
        //cout<<"blk 8"<<endl;
        //Lane_detector::display(speed_official,first_reading_available_flag,image_number);

        if ((speed_read_flag==1) || (speed_read_flag_1==1))
        {
            cout<<"speed_official: "<<speed_official<<endl;
            //cin>>any;
        }
        //cout<<"blk 9"<<endl;


    if (speed_1_hold==1)   
    {
         count_1_hold=count_1_hold+1;
         cout<<"count_1_hold: "<<count_1_hold<<endl;
    }
    if (speed_1_hold_due_to_delta_official==1)   
    {
         count_1_hold_due_to_delta_official=count_1_hold_due_to_delta_official+1;
         cout<<"count_1_hold_due_to_delta_official: "<<count_1_hold_due_to_delta_official<<endl;
    }

    if (speed_hold==1)
    {
        count_hold=count_hold+1;
        cout<<"count_hold: "<<count_hold<<endl; 
    }
    if (speed_hold_due_to_delta_official==1)
    {
        count_hold_due_to_delta_official=count_hold_due_to_delta_official+1;
        cout<<"count_hold: "<<count_hold_due_to_delta_official<<endl; 
    }

    //} This was the end of previous for loop

    //    Mat img = imread(file_name.c_str(), IMREAD_GRAYSCALE);
    //    Mat img3 = imread(file_name.c_str());
    //    Mat img2; 
    //    Mat img4;
    //    Mat img6;
    //    Mat img7;

    //    img(Rect(0,1500,3840-0,1800-1500)).copyTo(img2);
    //    img3(Rect(0,1500,3840-0,1800-1500)).copyTo(img4);

        
    //    h1 = img2.rows;
    //    w1 = img2.cols;

    //    img6 = img4.clone();

    //    count_lanes=0;

    //    //#Initialization on every frame
       
    //    vector<double> mux_lane_vec(40,0);
    //    vector<double> muy_lane_vec(40,0);
    //    vector<double> base_ptx_lane_vec(40,0);
    //    vector<double> base_pty_lane_vec(40,0);
    //    vector<double> angle_lanes(40,0);


    //    //######## LSD ALGORITHM ########    

    //    //#LSD line segment detection
    //    ls->detect(img2, lines_std); //#lines_std holds the lines that have been detected by LSD

    //    //cout<< lines_std[0][0] << endl;
    //    cout<< lines_std.size()<<endl;

    //    for (int k=0; k<lines_std.size(); k++)
    //    {
    //       x0 = int(round(lines_std[k][0]));
    //       y0 = int(round(lines_std[k][1]));
    //       x1 = int(round(lines_std[k][2]));
    //       y1 = int(round(lines_std[k][3]));
    //       //cv2.line(img2, (x0, y0), (x1,y1), 255, 1, cv2.LINE_AA)  #we are redrawing the lines to emphasize their borders
    //       line(img2, Point(x0,y0), Point(x1,y1), Scalar(255,255,255), 2, 1);

    //    }


    //    cv::resize(img6, img7, cv::Size(3840,2880), 0, 0);

    //    namedWindow("frame", WINDOW_NORMAL);
    //    imshow("frame",img7);
    //    waitKey(1);

    //}


//} //end of while    

}
