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



void Speed_estimator::Speed_estimator_update(string file_name, int image_number)
{

 

        Mat img = imread(file_name.c_str(), IMREAD_GRAYSCALE);
        Mat img3 = imread(file_name.c_str());
        Mat img2; 
        Mat img4;
        Mat img6;

        img(Rect(0,940,2704-0,1240-940)).copyTo(img2);
        img3(Rect(0,940,2704-0,1240-940)).copyTo(img4);

        
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

        Speed_estimator::absolute_speed_estimation( speed, road_nomark, capture_frameindex_for_speed, frameindex_for_speed, white_mark_hit, speed_read_flag, count_scanned_lines_reverse_for_speed, count_scanned_lines_for_speed, offset, offset_at_end_of_mark, offset_at_middle_of_mark, muy_lane_vec_final2_speed_permanent, mux_lane_vec_final2_speed_permanent, base_ptx_lane_vec_final2_speed_permanent, base_pty_lane_vec_final2_speed_permanent, h1, frameindex_for_speed_previous, image_number, count_scanned_lines_reverse_for_speed_previous, count_scanned_lines_for_speed_previous, offset_adjustment, range_adjustment_left, range_adjustment_right, img6, img2);

        //cout<<"speed_read_flag "<<speed_read_flag<<endl;
        //cout<<"white_mark_hit "<<white_mark_hit<<endl;
        //cout<<"road_nomark "<<road_nomark<<endl;
        //cout<<"count_road_nomark "<<count_road_nomark<<endl;
        //cout<<"frameindex_for_speed "<<frameindex_for_speed<<endl;
        //cin>>any;

        if (speed_read_flag==1)
        {
            speed_official=speed;
            first_reading_available_flag=1;
            //cout<<"speed_read_flag"<<speed_read_flag<<endl;
            //cout<<"frameindex_for_speed "<<frameindex_for_speed<<endl;
            //cout<<"frameindex_for_speed_previous: "<<frameindex_for_speed_previous<<endl;
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
            frameindex_for_speed_previous=frameindex_for_speed;
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

        Speed_estimator::absolute_speed_estimation( speed_1, road_nomark_1, capture_frameindex_for_speed_1, frameindex_for_speed_1, white_mark_hit_1, speed_read_flag_1, count_scanned_lines_reverse_for_speed_1, count_scanned_lines_for_speed_1, offset_1, offset_at_end_of_mark_1, offset_at_middle_of_mark_1, muy_lane_vec_final1_speed_permanent, mux_lane_vec_final1_speed_permanent, base_ptx_lane_vec_final1_speed_permanent, base_pty_lane_vec_final1_speed_permanent, h1, frameindex_for_speed_previous_1, image_number, count_scanned_lines_reverse_for_speed_previous_1, count_scanned_lines_for_speed_previous_1, offset_adjustment_1, range_adjustment_left_1, range_adjustment_right_1, img6, img2);

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
            frameindex_for_speed_previous_1=frameindex_for_speed_1;
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


        cout<<"speed_official: "<<speed_official<<endl;

        Mat img7;

        resize(img6, img7, Size(3840,2880), 0, 0, CV_INTER_LINEAR);

        if (first_reading_available_flag!=0)
        {
            speed_text="Speed: "+to_string(int(speed_official))+" miles/hr";
            putText(img7,speed_text.c_str(), Point(250,150), FONT_HERSHEY_SIMPLEX, 4, Scalar(255,255,255),2,LINE_AA);
        }
        cout<<"TIME: "<<image_number/30<<endl;
        namedWindow("frame", WINDOW_NORMAL);
        imshow("frame",img7);
        waitKey(1);
    

}

Speed_estimator::~Speed_estimator()
{
}
