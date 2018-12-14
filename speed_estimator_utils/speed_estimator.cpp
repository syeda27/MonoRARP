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

/*
 * This main speed estimator function was brought about for the purposes of
 * testing the python-> c++ interface, because adding extern here was the only
 * way I could get it to work. I am probably missing something dumb, like how
 * to link in the g++ commands, but at least I got it to work. Appreciate any help --djp42
 *
 * This allows us to not have so many links at compile time.
 * TODO: clean everythin!!!
 */

Speed_estimator::Speed_estimator(bool do_display)
{
    // TODO: args
    display=do_display;

    white_mark_hit=0;
    count_road_nomark=0;
    capture_frameindex_for_speed=0;
    frameindex_for_speed_previous=0;
    frameindex_for_speed=0;
    count_scanned_lines_reverse_for_speed_previous=0;
    count_scanned_lines_reverse_for_speed=0;
    count_scanned_lines_for_speed_previous=0;
    count_scanned_lines_for_speed=0;
    road_nomark=0;


    white_mark_hit_1=0;
    count_road_nomark_1=0;
    capture_frameindex_for_speed_1=0;
    frameindex_for_speed_previous_1=0;
    frameindex_for_speed_1=0;
    count_scanned_lines_reverse_for_speed_previous_1=0;
    count_scanned_lines_reverse_for_speed_1=0;
    count_scanned_lines_for_speed_previous_1=0;
    count_scanned_lines_for_speed_1=0;
    road_nomark_1=0;

    base_ptx_lane_vec_final1=0;
    base_pty_lane_vec_final1=0;
    mux_lane_vec_final1=0;
    muy_lane_vec_final1=0;

    base_ptx_lane_vec_final2=0;
    base_pty_lane_vec_final2=0;
    mux_lane_vec_final2=0;
    muy_lane_vec_final2=0;

    mux_lane_vec_final1_previous=0;
    muy_lane_vec_final1_previous=0;
    base_ptx_lane_vec_final1_previous=0;
    base_pty_lane_vec_final1_previous=0;

    mux_lane_vec_final2_previous=0;
    muy_lane_vec_final2_previous=0;
    base_ptx_lane_vec_final2_previous=0;
    base_pty_lane_vec_final2_previous=0;


    initial_frame_was_processed_flag=0;

    first_reading_available_flag=0;

    parmanent_positioning=0;

    offset=0;
    offset_at_end_of_mark=0;
    offset_at_middle_of_mark=0;
    offset_1=0;
    offset_at_end_of_mark_1=0;
    offset_at_middle_of_mark_1=0;

    offset_adjustment=0;
    offset_adjustment_1=0;
    count_marks_detected_for_adjustment=0;
    count_marks_detected_for_adjustment_b=0;
    count_marks_detected_for_adjustment_c=0;
    count_marks_detected_for_adjustment_1=0;
    count_marks_detected_for_adjustment_b_1=0;
    count_marks_detected_for_adjustment_c_1=0;
    offset_adjustment_acc=0;
    offset_adjustment_acc_b=0;
    offset_adjustment_acc_c=0;
    offset_adjustment_acc_1=0;
    offset_adjustment_acc_b_1=0;
    offset_adjustment_acc_c_1=0;
    count_adjustments=0;
    count_adjustments_b=0;
    count_adjustments_c=0;
    count_adjustments_1=0;
    count_adjustments_b_1=0;
    count_adjustments_c_1=0;
    exclude=0;
    exclude_b=0;
    exclude_c=0;
    exclude_1=0;
    exclude_b_1=0;
    exclude_c_1=0;
    range_adjustment_left=0;
    range_adjustment_right=0;
    range_adjustment_left_1=0;
    range_adjustment_right_1=0;

    Left_lane_available_display=0;
    Right_lane_available_display=0;


}


Speed_estimator::~Speed_estimator()
{
}

double Speed_estimator::get_speed()
{
    return speed_official;
}

void
 Speed_estimator::absolute_speed_estimation_time(double &speed,int &road_nomark,int &capture_frameindex_for_speed,double &frametime,int &white_mark_hit,int &speed_read_flag,int &count_scanned_lines_reverse_for_speed,int &count_scanned_lines_for_speed,int &offset,int &offset_at_end_of_mark,int &offset_at_middle_of_mark,double muy_lane_vec_speed,double mux_lane_vec_speed,double base_ptx_lane_vec_speed,double base_pty_lane_vec_speed,int h1,double frametime_previous,double image_time,int count_scanned_lines_reverse_for_speed_previous,int count_scanned_lines_for_speed_previous,double offset_adjustment,int range_adjustment_left,int range_adjustment_right,Mat &img6,Mat &img2)
{

   double Lintersection;
   double x2_lane;
   double value;
   double value_scan;
   double previous;
   double x2_lane_scan;
   double previous_scan;
   double time_b;
   double correction_factor;
   double correction_factor2;
   int found_end_of_mark;
   int out_of_the_mark;
   int count_scanned_lines;
   int count_scanned_lines_reverse;
   int found_end_of_mark_reverse;
   int out_of_the_mark_reverse;

   speed_read_flag=0;
   speed=0;
   offset=0;
   offset_at_end_of_mark=0;
   offset_at_middle_of_mark=-1000;
   int any;

   //#Intersection between the Lane and the marker
   Lintersection=(0.7*h1-base_pty_lane_vec_speed)/muy_lane_vec_speed;
   x2_lane=base_ptx_lane_vec_speed+Lintersection*mux_lane_vec_speed+offset_adjustment;

   line(img6, Point(int(x2_lane-50), int(0.7*h1)), Point(int(x2_lane+50), int(0.7*h1)), Scalar(255,0,0), 1, LINE_AA);

   //#Scanning horizontally over the marker and detect high transition
   road_nomark=1;

   for (int k5=-120+range_adjustment_left; k5<120; k5++)
   {

       value = (int)img2.at<uchar>(int(0.7*h1),int(x2_lane+k5));
       //cout<<"value: "<<value<<endl;
       //cout<<"int(0.7*h1): "<<int(0.7*h1)<<endl;
       //cout<<"int(x2_lane+k5): "<<int(x2_lane+k5)<<endl;
       //cout<<"range_adjustment_left: "<<range_adjustment_left<<endl;
       //cin>>any;

       if (k5>-120+range_adjustment_left)
       {
            //#if value>1.2*previous:
            if (value>1.1*previous)
            {
                road_nomark=0;

                offset=k5;

                //#scanning upwards to find the end of the mark
                found_end_of_mark=0;
                count_scanned_lines=0;

                for (int k6=0; k6<100; k6++)
                {
                    Lintersection=(0.7*h1-k6-base_pty_lane_vec_speed)/muy_lane_vec_speed;
                    x2_lane_scan=base_ptx_lane_vec_speed+Lintersection*mux_lane_vec_speed+offset_adjustment;

                    line(img6, Point(int(x2_lane_scan-50), int(0.7*h1-k6)), Point(int(x2_lane_scan+50), int(0.7*h1-k6)), Scalar(255,0,0), 1, LINE_AA);

                    count_scanned_lines=count_scanned_lines+1;

                    out_of_the_mark=1;

                    for (int k7=-120+range_adjustment_left; k7<120; k7++)
                    {

                        value_scan = (int)img2.at<uchar>(int(0.7*h1-k6),int(x2_lane_scan+k7));

                        if (k7>-120+range_adjustment_left)
                        {
                            //#if value_scan>1.2*previous_scan:
                            if (value_scan>1.1*previous_scan)
                            {
                                out_of_the_mark=0;
                                offset_at_end_of_mark=k7;

                                if (count_scanned_lines==20)
                                {
                                    offset_at_middle_of_mark=k7;
                                    //#print("coord end mark: ",int(0.7*h1-k6))
                                    //#print("count_scanned_lines: ",count_scanned_lines)
                                    //#cv2.line(img6, (int(x2_lane_scan-50), int(0.7*h1-k6)), (int(x2_lane_scan-100), int(0.7*h1-k6)), (0,0,255), 2, cv2.LINE_AA)
                                    //#cv2.namedWindow('img',cv2.WINDOW_NORMAL)
                                    //#cv2.imshow('img',img6 )
                                    //#cv2.waitKey(0)
                                }

                                break;
                            }

                        }

                        previous_scan=value_scan;

                    }

                    if (out_of_the_mark==1)
                    {
                        break;
                    }

                }// end of k6


                //#finding the beginning of the lane (we scan in reverse when the mark is not set properly at beggining of road mark. Reverse direction is scanning downwards)
                found_end_of_mark_reverse=0;
                count_scanned_lines_reverse=0;

                for (int k6=0; k6<100; k6++)
                {
                    Lintersection=(0.7*h1+k6-base_pty_lane_vec_speed)/muy_lane_vec_speed;
                    x2_lane_scan=base_ptx_lane_vec_speed+Lintersection*mux_lane_vec_speed+offset_adjustment;

                    line(img6, Point(int(x2_lane_scan-50), int(0.7*h1+k6)), Point(int(x2_lane_scan+50), int(0.7*h1+k6)), Scalar(0,0,255), 1, LINE_AA);
                    count_scanned_lines_reverse=count_scanned_lines_reverse+1;

                    out_of_the_mark_reverse=1;

                    //#for k7 in range(-20,20):
                    for (int k7=-120+range_adjustment_left; k7<120; k7++)
                    {

                        if (0.7*h1+k6>=h1)
                        {
                            break;
                        }
                        else
                        {
                            value_scan = (int)img2.at<uchar>(int(0.7*h1+k6),int(x2_lane_scan+k7));
                        }


                        if (k7>-120+range_adjustment_left)
                        {
                            //#if value_scan>1.2*previous_scan:
                            if (value_scan>1.1*previous_scan)
                            {
                                out_of_the_mark_reverse=0;
                                //#print("coord end mark: ",int(0.7*h1+k6))
                                //#print("count_scanned_lines_reverse: ",count_scanned_lines_reverse)
                                //#cv2.line(img6, (int(x2_lane_scan-50), int(0.7*h1+k6)), (int(x2_lane_scan-100), int(0.7*h1+k6)), (0,0,255), 2, cv2.LINE_AA)
                                //#cv2.namedWindow('img',cv2.WINDOW_NORMAL)
                                //#cv2.imshow('img',img6 )
                                //#cv2.waitKey(0)
                                break;
                            }
                        }

                        previous_scan=value_scan;

                    }

                    if (out_of_the_mark_reverse==1)
                    {
                        break;
                    }
                }

                //#if capture_frameindex_for_speed==0 and count_scanned_lines>=20:
                if ((capture_frameindex_for_speed==0) && (count_scanned_lines+count_scanned_lines_reverse>=40))
                {
                    white_mark_hit=1;
                    frametime=image_time;
                    count_scanned_lines_reverse_for_speed=count_scanned_lines_reverse;
                    count_scanned_lines_for_speed=count_scanned_lines;
                    capture_frameindex_for_speed=1;

                    if (frametime_previous!=0)
                    {
                        time_b=double(frametime-frametime_previous);
                        //cout<<"frametime "<<frametime<<endl;
                        //cout<<"frametime_previous "<<frametime_previous<<endl;
                        //cout<<"time "<<time_b<<endl;
                        //cin>>any;
                        speed=(40/time_b)*0.682;
                        speed_read_flag=1;
                        //#print("count_scanned_lines+count_scanned_lines_reverse: ",count_scanned_lines+count_scanned_lines_reverse)
                        //#input("lines")
                        if (count_scanned_lines+count_scanned_lines_reverse>40) //#and count_scanned_lines>20: #meaning correct road mark
                        {
                            correction_factor=(count_scanned_lines_reverse/(count_scanned_lines+count_scanned_lines_reverse))*10;
                            correction_factor2=(count_scanned_lines_reverse_for_speed_previous/(count_scanned_lines_reverse_for_speed_previous+count_scanned_lines_for_speed_previous))*10;
                            speed=((40+correction_factor-correction_factor2)/time_b)*0.682;
                        }
                    }
                }
                break;
            }//if (value>1.1*previous)
       }//if (k5>-120+range_adjustment_left)
       previous=value;
   }// end of for k5
}



/**
 * A version of the update that i have deemed more useful.
 * It accepts just an image and the time the image was recorded, as opposed
 * to a file name which forces it to load from a file and not work with a source.
 * Args:
 *   img3: color version on an image
 *   time: double of the time in seconds that the image was recorded
 */
void Speed_estimator::Speed_estimator_update(Mat img3, double image_time)
{
        Mat img;
        cv::cvtColor(img3, img, CV_BGR2GRAY);
        Mat img2;
        Mat img4;
        Mat img6;

        int h = img3.rows;
        int w = img3.cols;

        //int subframe_x_start = 0;
        int subframe_y_start = 940;
        int subframe_y_end = 1240;
        if (h < 1240) {
          subframe_y_start = 350;
          subframe_y_end = 550;
        }
        //int subframe_y_start = 940;
        //int subframe_y_end = 1240;

        img(Rect(0, subframe_y_start, w, subframe_y_end-subframe_y_start)).copyTo(img2);
        img3(Rect(0, subframe_y_start, w, subframe_y_end-subframe_y_start)).copyTo(img4);
        // og height of ~2100, w of 2704
        //img(Rect(0,940,2704-0,1240-940)).copyTo(img2);
        //img3(Rect(0,940,2704-0,1240-940)).copyTo(img4);

        h1 = img2.rows;
        w1 = img2.cols;

        img6 = img4.clone();

        //######## ABSOLUTE SPEED DETERMINATION ########

        muy_lane_vec_final2_speed_permanent_34=-0.5199735932802914;
        mux_lane_vec_final2_speed_permanent_34=0.8541686919536207;
        base_ptx_lane_vec_final2_speed_permanent_34=1193.0;
        base_pty_lane_vec_final2_speed_permanent_34=94.5;

        muy_lane_vec_final2_speed_permanent_36= -0.5070201265633938;
        mux_lane_vec_final2_speed_permanent_36=  0.8619342151577695;
        base_ptx_lane_vec_final2_speed_permanent_36=  1225.0;
        base_pty_lane_vec_final2_speed_permanent_36=  138.0;

        muy_lane_vec_final2_speed_permanent= (muy_lane_vec_final2_speed_permanent_34+muy_lane_vec_final2_speed_permanent_36)/2;
        mux_lane_vec_final2_speed_permanent= (mux_lane_vec_final2_speed_permanent_34+mux_lane_vec_final2_speed_permanent_36)/2;
        base_ptx_lane_vec_final2_speed_permanent= (base_ptx_lane_vec_final2_speed_permanent_34+base_ptx_lane_vec_final2_speed_permanent_36)/2;
        base_pty_lane_vec_final2_speed_permanent= (base_pty_lane_vec_final2_speed_permanent_34+base_pty_lane_vec_final2_speed_permanent_36)/2;


        //#Speed on left track (group2)
        speed_read_flag=0;

        Speed_estimator::absolute_speed_estimation_time( speed, road_nomark, capture_frameindex_for_speed, frametime, white_mark_hit, speed_read_flag, count_scanned_lines_reverse_for_speed, count_scanned_lines_for_speed, offset, offset_at_end_of_mark, offset_at_middle_of_mark, muy_lane_vec_final2_speed_permanent, mux_lane_vec_final2_speed_permanent, base_ptx_lane_vec_final2_speed_permanent, base_pty_lane_vec_final2_speed_permanent, h1, frametime_previous, image_time, count_scanned_lines_reverse_for_speed_previous, count_scanned_lines_for_speed_previous, offset_adjustment, range_adjustment_left, range_adjustment_right, img6, img2);

        //cout<<"speed_read_flag "<<speed_read_flag<<endl;
        //cout<<"white_mark_hit "<<white_mark_hit<<endl;
        //cout<<"road_nomark "<<road_nomark<<endl;
        //cout<<"count_road_nomark "<<count_road_nomark<<endl;
        //cout<<"frametime "<<frametime<<endl;
        //cin>>any;

        if (speed_read_flag==1)
        {
            speed_official=speed;
            first_reading_available_flag=1;
            //cout<<"speed_read_flag"<<speed_read_flag<<endl;
            //cout<<"frametime "<<frametime<<endl;
            //cout<<"frametime_previous: "<<frametime_previous<<endl;
            //cin>>any;
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
            frametime_previous=frametime;
            count_scanned_lines_reverse_for_speed_previous=count_scanned_lines_reverse_for_speed;
            count_scanned_lines_for_speed_previous=count_scanned_lines_for_speed;
        }

        if (speed_read_flag==1)
        {
            //print("offset: ",offset)
            //print("count_scanned_lines_reverse_for_speed+count_scanned_lines_for_speed: ",count_scanned_lines_reverse_for_speed+count_scanned_lines_for_speed)
            //#input("offset")

            if ((count_scanned_lines_reverse_for_speed+count_scanned_lines_for_speed<60) && (count_scanned_lines_reverse_for_speed+count_scanned_lines_for_speed>=51) && (exclude==0))
            {
                if (abs(offset)<70)
                {
                    count_marks_detected_for_adjustment=count_marks_detected_for_adjustment+1;
                    offset_adjustment_acc=offset_adjustment_acc+offset;
                    //cout<<"offset_considered:"<<offset<<endl;
                    //print("offset_considered:",offset)
                    //#input("conidered")
                }
            }
            else
            {
                if ((count_scanned_lines_reverse_for_speed+count_scanned_lines_for_speed<50) && (count_scanned_lines_reverse_for_speed+count_scanned_lines_for_speed>=40) && (exclude_b==0))
                {
                  if (abs(offset)<100)
                  {
                    count_marks_detected_for_adjustment_b=count_marks_detected_for_adjustment_b+1;
                    offset_adjustment_acc_b=offset_adjustment_acc_b+offset;
                    //cout<<"offset_considered:"<<offset<<endl;
                    //print("offset_considered:",offset)
                    //#input("conidered_b")
                  }
                }
                else
                {
                    if ((count_scanned_lines_reverse_for_speed+count_scanned_lines_for_speed<=95) && (count_scanned_lines_reverse_for_speed+count_scanned_lines_for_speed>=90) && (exclude_c==0))
                    {
                        if (abs(offset)<100)
                        {
                            count_marks_detected_for_adjustment_c=count_marks_detected_for_adjustment_c+1;
                            offset_adjustment_acc_c=offset_adjustment_acc_c+offset;
                            cout<<"offset_considered:"<<offset<<endl;
                            //print("offset_considered:",offset)
                            //#input("conidered_c")
                        }
                    }
                }
            }


            if (count_marks_detected_for_adjustment==2)
            {
                offset_adjustment=offset_adjustment+0.6*(offset_adjustment_acc/count_marks_detected_for_adjustment); //#moving conservatively;
                count_marks_detected_for_adjustment=0;
                offset_adjustment_acc=0;
                count_adjustments=count_adjustments+1;
                //print("offset_adjustment: ",offset_adjustment)
                //#input("adjustment")
            }

            if (count_marks_detected_for_adjustment_b==2)
            {
                offset_adjustment=offset_adjustment+0.6*(offset_adjustment_acc_b/count_marks_detected_for_adjustment_b); //#moving conservatively;
                count_marks_detected_for_adjustment_b=0;
                offset_adjustment_acc_b=0;
                count_adjustments_b=count_adjustments_b+1;
                //print("offset_adjustment: ",offset_adjustment)
                //#input("adjustment_b")
            }

            if (count_marks_detected_for_adjustment_c==2)
            {
                offset_adjustment=offset_adjustment+0.6*(offset_adjustment_acc_c/count_marks_detected_for_adjustment_c); //#moving conservatively;
                count_marks_detected_for_adjustment_c=0;
                offset_adjustment_acc_c=0;
                count_adjustments_c=count_adjustments_c+1;
                cout<<"offset_adjustment: "<<offset_adjustment<<endl;
                //print("offset_adjustment: ",offset_adjustment)
                //#input("adjustment_c")
            }

            if (count_adjustments==4)
            {
                exclude_b=1;
                exclude_c=1;
                //#input("exclude_b and c")
            }
            if (count_adjustments_b==4)
            {
                exclude=1;
                exclude_c=1;
                //#input("exclude and c")
            }
            if (count_adjustments_c==4)
            {
                exclude=1;
                exclude_b=1;
                //#input("exclude and b")
            }

            if (count_adjustments==4)
            {
                range_adjustment_left=60; //#decrease left range by 40
                //#input("range adjusted")
            }

            if (count_adjustments_b==4)
            {
                range_adjustment_left=60; //#decrease left range by 40
                //#input("range_b adjusted")
            }

            if (count_adjustments_c==4)
            {
                range_adjustment_left=60; //#decrease left range by 40
                //#input("range_c adjusted")
            }

        }


        muy_lane_vec_final1_speed_permanent_34=-0.49613893835683387;
        mux_lane_vec_final1_speed_permanent_34=-0.8682431421244593;
        base_ptx_lane_vec_final1_speed_permanent_34=1468.0;
        base_pty_lane_vec_final1_speed_permanent_34=55.0;

        muy_lane_vec_final1_speed_permanent_36=-0.49613893835683387;
        mux_lane_vec_final1_speed_permanent_36=-0.8682431421244593;
        base_ptx_lane_vec_final1_speed_permanent_36=1631.0;
        base_pty_lane_vec_final1_speed_permanent_36=192.0;

        muy_lane_vec_final1_speed_permanent= (muy_lane_vec_final1_speed_permanent_34+muy_lane_vec_final1_speed_permanent_36)/2;
        mux_lane_vec_final1_speed_permanent= (mux_lane_vec_final1_speed_permanent_34+mux_lane_vec_final1_speed_permanent_36)/2;
        base_ptx_lane_vec_final1_speed_permanent= (base_ptx_lane_vec_final1_speed_permanent_34+base_ptx_lane_vec_final1_speed_permanent_36)/2;
        base_pty_lane_vec_final1_speed_permanent= (base_pty_lane_vec_final1_speed_permanent_34+base_pty_lane_vec_final1_speed_permanent_36)/2;


        //#Speed on right track (group1)
        speed_read_flag_1=0;

        Speed_estimator::absolute_speed_estimation_time( speed_1, road_nomark_1, capture_frameindex_for_speed_1, frametime_1, white_mark_hit_1, speed_read_flag_1, count_scanned_lines_reverse_for_speed_1, count_scanned_lines_for_speed_1, offset_1, offset_at_end_of_mark_1, offset_at_middle_of_mark_1, muy_lane_vec_final1_speed_permanent, mux_lane_vec_final1_speed_permanent, base_ptx_lane_vec_final1_speed_permanent, base_pty_lane_vec_final1_speed_permanent, h1, frametime_previous_1, image_time, count_scanned_lines_reverse_for_speed_previous_1, count_scanned_lines_for_speed_previous_1, offset_adjustment_1, range_adjustment_left_1, range_adjustment_right_1, img6, img2);

        //cout<<"speed_read_flag "<<speed_read_flag<<endl;
        //cout<<"white_mark_hit "<<white_mark_hit<<endl;
        //cout<<"road_nomark "<<road_nomark<<endl;
        //cout<<"count_road_nomark "<<count_road_nomark<<endl;

        if (speed_read_flag_1==1)
        {
            speed_official=speed_1;
            first_reading_available_flag=1;
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
            frametime_previous_1=frametime_1;
            count_scanned_lines_reverse_for_speed_previous_1=count_scanned_lines_reverse_for_speed_1;
            count_scanned_lines_for_speed_previous_1=count_scanned_lines_for_speed_1;
        }

        if (speed_read_flag_1==1)
        {
            //print("offset: ",offset)
            //print("count_scanned_lines_reverse_for_speed+count_scanned_lines_for_speed: ",count_scanned_lines_reverse_for_speed+count_scanned_lines_for_speed)
            //#input("offset")

            if ((count_scanned_lines_reverse_for_speed_1+count_scanned_lines_for_speed_1<60) && (count_scanned_lines_reverse_for_speed_1+count_scanned_lines_for_speed_1>=51) && (exclude_1==0))
            {
                if (abs(offset_1)<70)
                {
                    count_marks_detected_for_adjustment_1=count_marks_detected_for_adjustment_1+1;
                    offset_adjustment_acc_1=offset_adjustment_acc_1+offset_1;
                    //cout<<"offset_considered:"<<offset<<endl;
                    //print("offset_considered:",offset)
                    //#input("conidered")
                }
            }
            else
            {
                if ((count_scanned_lines_reverse_for_speed_1+count_scanned_lines_for_speed_1<50) && (count_scanned_lines_reverse_for_speed_1+count_scanned_lines_for_speed_1>=40) && (exclude_b_1==0))
                {
                  if (abs(offset_1)<100)
                  {
                    count_marks_detected_for_adjustment_b_1=count_marks_detected_for_adjustment_b_1+1;
                    offset_adjustment_acc_b_1=offset_adjustment_acc_b_1+offset_1;
                    //cout<<"offset_considered:"<<offset<<endl;
                    //print("offset_considered:",offset)
                    //#input("conidered_b")
                  }
                }
                else
                {
                    if ((count_scanned_lines_reverse_for_speed_1+count_scanned_lines_for_speed_1<=95) && (count_scanned_lines_reverse_for_speed_1+count_scanned_lines_for_speed_1>=90) && (exclude_c_1==0))
                    {
                        if (abs(offset_1)<100)
                        {
                            count_marks_detected_for_adjustment_c_1=count_marks_detected_for_adjustment_c_1+1;
                            offset_adjustment_acc_c_1=offset_adjustment_acc_c_1+offset_1;
                            cout<<"offset_considered_1:"<<offset_1<<endl;
                            //print("offset_considered:",offset)
                            //#input("conidered_c")
                        }
                    }
                }
            }


            if (count_marks_detected_for_adjustment_1==2)
            {
                offset_adjustment_1=offset_adjustment_1+0.6*(offset_adjustment_acc_1/count_marks_detected_for_adjustment_1); //#moving conservatively;
                count_marks_detected_for_adjustment_1=0;
                offset_adjustment_acc_1=0;
                count_adjustments_1=count_adjustments+1;
                //print("offset_adjustment: ",offset_adjustment)
                //#input("adjustment")
            }

            if (count_marks_detected_for_adjustment_b_1==2)
            {
                offset_adjustment_1=offset_adjustment_1+0.6*(offset_adjustment_acc_b_1/count_marks_detected_for_adjustment_b_1); //#moving conservatively;
                count_marks_detected_for_adjustment_b_1=0;
                offset_adjustment_acc_b_1=0;
                count_adjustments_b_1=count_adjustments_b_1+1;
                //print("offset_adjustment: ",offset_adjustment)
                //#input("adjustment_b")
            }

            if (count_marks_detected_for_adjustment_c_1==2)
            {
                offset_adjustment_1=offset_adjustment_1+0.6*(offset_adjustment_acc_c_1/count_marks_detected_for_adjustment_c_1); //#moving conservatively;
                count_marks_detected_for_adjustment_c_1=0;
                offset_adjustment_acc_c_1=0;
                count_adjustments_c_1=count_adjustments_c_1+1;
                cout<<"offset_adjustment_1: "<<offset_adjustment_1<<endl;
                //print("offset_adjustment: ",offset_adjustment)
                //#input("adjustment_c")
            }

            if (count_adjustments_1==4)
            {
                exclude_b_1=1;
                exclude_c_1=1;
                //#input("exclude_b and c")
            }
            if (count_adjustments_b_1==4)
            {
                exclude_1=1;
                exclude_c_1=1;
                //#input("exclude and c")
            }
            if (count_adjustments_c_1==4)
            {
                exclude_1=1;
                exclude_b_1=1;
                //#input("exclude and b")
            }

            if (count_adjustments_1==4)
            {
                range_adjustment_right_1=60; //#decrease left range by 40
                //#input("range adjusted")
            }

            if (count_adjustments_b_1==4)
            {
                range_adjustment_right_1=60; //#decrease left range by 40
                //#input("range_b adjusted")
            }

            if (count_adjustments_c_1==4)
            {
                range_adjustment_right_1=60; //#decrease left range by 40
                //#input("range_c adjusted")
            }

        }

        if (display == true) {

          cout<<"speed_official: "<<speed_official<<endl;

          Mat img7;

          resize(img6, img7, Size(3840,2880), 0, 0, CV_INTER_LINEAR);

          if (first_reading_available_flag!=0)
          {
              speed_text="Speed: "+to_string(int(speed_official))+" miles/hr";
              putText(img7,speed_text.c_str(), Point(250,150), FONT_HERSHEY_SIMPLEX, 4, Scalar(255,255,255),2,LINE_AA);
          }
          cout<<"TIME: "<<image_time<<endl;
          namedWindow("frame", WINDOW_NORMAL);
          imshow("frame",img7);
          waitKey(1);
      }
}

extern "C" {
    Speed_estimator* Speed_estimator_new(bool display) {
        return new Speed_estimator(display);
    }
    void Speed_estimator_hello_world(Speed_estimator* speed_estimator) {
        speed_estimator->hello_world();
    }
    void Speed_estimator_update(
          Speed_estimator* speed_estimator,
          void* image,
          int height,
          int width,
          double time_s) {
        //const unsigned char * indata = (unsigned char *) image;
        //cout << indata[1] << endl;
        Mat mat = Mat(height, width, CV_8UC(3), image);
        speed_estimator->Speed_estimator_update(mat, time_s);
    }
    double Speed_estimator_get_speed(Speed_estimator* speed_estimator) {
        return speed_estimator->get_speed();
    }
}
