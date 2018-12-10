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





void  
 Speed_estimator::absolute_speed_estimation(double &speed,int &road_nomark,int &capture_frameindex_for_speed,int &frameindex_for_speed,int &white_mark_hit,int &speed_read_flag,int &count_scanned_lines_reverse_for_speed,int &count_scanned_lines_for_speed,int &offset,int &offset_at_end_of_mark,int &offset_at_middle_of_mark,double muy_lane_vec_speed,double mux_lane_vec_speed,double base_ptx_lane_vec_speed,double base_pty_lane_vec_speed,int h1,int frameindex_for_speed_previous,int image_number,int count_scanned_lines_reverse_for_speed_previous,int count_scanned_lines_for_speed_previous,double offset_adjustment,int range_adjustment_left,int range_adjustment_right,Mat &img6,Mat &img2)    
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
                    frameindex_for_speed=image_number;
                    count_scanned_lines_reverse_for_speed=count_scanned_lines_reverse;
                    count_scanned_lines_for_speed=count_scanned_lines;
                    capture_frameindex_for_speed=1;
                    
                    if (frameindex_for_speed_previous!=0)
                    {
                        
                        time_b=double(frameindex_for_speed-frameindex_for_speed_previous)/30;
                        //cout<<"frameindex_for_speed "<<frameindex_for_speed<<endl;
                        //cout<<"frameindex_for_speed_previous "<<frameindex_for_speed_previous<<endl;
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
