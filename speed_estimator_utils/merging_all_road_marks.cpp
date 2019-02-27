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

void Lane_detector::merging_all_road_marks()
{

    //#Merging all found lanes (without repetition) into two lanes
    //#first pick the first lane
    //print("count_tracked_lanes: ",count_tracked_lanes2)
    //cout<<"entered merging"<<endl;    

    //vector<int> lane_group1(40,0); //#one group of lanes with positive angle
    //vector<int> lane_group2(40,0); //#one group of lanes with negative angle

    //cout<<"this part"<<endl;  
    double mux_lane_vec_acc;
    double muy_lane_vec_acc;
    double base_ptx_lane_vec_acc;
    double base_pty_lane_vec_acc;
    
    count_lane_group1=0;
    count_lane_group2=0;

    mux_lane_vec_final1=0;
    muy_lane_vec_final1=0;
    base_ptx_lane_vec_final1=0;
    base_pty_lane_vec_final1=0;

    mux_lane_vec_final2=0;
    muy_lane_vec_final2=0;
    base_ptx_lane_vec_final2=0;
    base_pty_lane_vec_final2=0;

    double angleA;
    double angleB;

    override_lane_group2=0;
    override_lane_group1=0;

    double Lintersection;
    double x1_lane;

    count_marks_filtered_grp2=0;
    count_marks_filtered_grp1=0;

    int override_break;
    
    
    //cout<<"count_tracked_lanes2 "<<count_tracked_lanes2<<endl;
    for (int lanes=0; lanes<count_tracked_lanes2; lanes++)
    {
        if (angle_lanes[lanes]>0)
        {
            //cout<<"count_lane_group1 "<<count_lane_group1<<endl;
            lane_group1[count_lane_group1]=int(lanes); //#storing indexes
            count_lane_group1=count_lane_group1+1;
        }
          
        else
        {
            //cout<<"count_lane_group2 "<<count_lane_group2<<endl;
            lane_group2[count_lane_group2]=int(lanes);
            count_lane_group2=count_lane_group2+1;
        }
    }
            
    //cout<<"this part2"<<endl;
    //cout<<"count_lane_group1: "<<count_lane_group1<<endl;
    //cin>>any;

    if (count_lane_group1>=2)
    {
        
         mux_lane_vec_acc=0;
         muy_lane_vec_acc=0;
         base_ptx_lane_vec_acc=0;
         base_pty_lane_vec_acc=0;
        
         for (int k4=0; k4<count_lane_group1; k4++)
         {
                
             //mux_lane_vec_acc=mux_lane_vec_acc+mux_lane_vec_aggregated[int(lane_group1[k4])];
             //muy_lane_vec_acc=muy_lane_vec_acc+muy_lane_vec_aggregated[int(lane_group1[k4])];
             //base_ptx_lane_vec_acc=base_ptx_lane_vec_acc+base_ptx_lane_vec_aggregated[int(lane_group1[k4])];
             //base_pty_lane_vec_acc=base_pty_lane_vec_acc+base_pty_lane_vec_aggregated[int(lane_group1[k4])];

             //restricted filtering here
             Lintersection=-base_pty_lane_vec_aggregated[int(lane_group1[k4])]/muy_lane_vec_aggregated[int(lane_group1[k4])];
             x1_lane=base_ptx_lane_vec_aggregated[int(lane_group1[k4])]+Lintersection*mux_lane_vec_aggregated[int(lane_group1[k4])];

             //cout<<"x1_lane a:"<<x1_lane<<endl;
             //cin>>any;

             if ((x1_lane>1300-15) && (x1_lane<1350+15))
             {

                 mux_lane_vec_acc=mux_lane_vec_acc+mux_lane_vec_aggregated[int(lane_group1[k4])];
                 muy_lane_vec_acc=muy_lane_vec_acc+muy_lane_vec_aggregated[int(lane_group1[k4])];
                 base_ptx_lane_vec_acc=base_ptx_lane_vec_acc+base_ptx_lane_vec_aggregated[int(lane_group1[k4])];
                 base_pty_lane_vec_acc=base_pty_lane_vec_acc+base_pty_lane_vec_aggregated[int(lane_group1[k4])];
                 count_marks_filtered_grp1=count_marks_filtered_grp1+1;

                 //cout<<"count_marks_filtered_grp1 a: "<<count_marks_filtered_grp1<<endl;
                 //cin>>any;

             }

             override_break=0;
             for (int k5=0; k5<count_lane_group1; k5++)
             {
                 if (k5!=k4)
                 {
                     angleA=180*(atan(muy_lane_vec_aggregated[int(lane_group1[k4])]/mux_lane_vec_aggregated[int(lane_group1[k4])])/3.14159);
                     angleB=180*(atan(muy_lane_vec_aggregated[int(lane_group1[k5])]/mux_lane_vec_aggregated[int(lane_group1[k5])])/3.14159);
                     if ((abs(angleA-angleB)<6) && (30<angleA) && (angleA<45))
                     {
                         //cout<<"aligned"<<endl;
                         //cout<<"angle: "<<180*(atan(muy_lane_vec_aggregated[int(lane_group1[k4])]/mux_lane_vec_aggregated[int(lane_group1[k4])])/3.14159)<<endl;
                         //override_lane_group1=1;
                         override_lane_group1_mux=(mux_lane_vec_aggregated[int(lane_group1[k5])]+mux_lane_vec_aggregated[int(lane_group1[k4])])/2;
                         override_lane_group1_muy=(muy_lane_vec_aggregated[int(lane_group1[k5])]+muy_lane_vec_aggregated[int(lane_group1[k4])])/2;
                         override_lane_group1_base_ptx=(base_ptx_lane_vec_aggregated[int(lane_group1[k5])]+base_ptx_lane_vec_aggregated[int(lane_group1[k4])])/2;
                         override_lane_group1_base_pty=(base_pty_lane_vec_aggregated[int(lane_group1[k5])]+base_pty_lane_vec_aggregated[int(lane_group1[k4])])/2;
                         //#intersecting with top of image  
                         Lintersection=-override_lane_group1_base_pty/override_lane_group1_muy;
                         x1_lane=override_lane_group1_base_ptx+Lintersection*override_lane_group1_mux;
                         //cout<<"x1_lane a: "<<x1_lane<<endl;
                         if ((x1_lane>1300) && (x1_lane<1365))
                         {
                            override_lane_group1=1;
                            override_break=1;
                            //cin>>any;
                            break;
                         }
                         //cin>>any;
                     }
                 }
             }

             if (override_break==1)
             {
                 break;
             }

         }


         if (override_lane_group1==0)
         {

             for (int k4=0 ; k4<count_lane_group1; k4++)
             {
                 override_break=0;
                 for (int k5=0; k5<count_lane_group1_previous; k5++)     
                 {
                 if (k5!=k4)
                 {
                     angleA=180*(atan(muy_lane_vec_aggregated[int(lane_group1[k4])]/mux_lane_vec_aggregated[int(lane_group1[k4])])/3.14159);
                     angleB=180*(atan(muy_lane_vec_aggregated_previous[int(lane_group1_previous[k5])]/mux_lane_vec_aggregated_previous[int(lane_group1_previous[k5])])/3.14159);
                     if ((abs(angleA-angleB)<6) && (30<angleA) && (angleA<45))
                     {
                         //cout<<"aligned"<<endl;
                         //cout<<"angle: "<<180*(atan(muy_lane_vec_aggregated[int(lane_group1[k4])]/mux_lane_vec_aggregated[int(lane_group1[k4])])/3.14159)<<endl;
                         //#intersecting with top of image        
                         //Lintersection=-base_pty_lane_vec_aggregated[int(lane_group2[k4])]/muy_lane_vec_aggregated[int(lane_group2[k4])];
                         //x1_lane=base_ptx_lane_vec_aggregated[int(lane_group2[k4])]+Lintersection*mux_lane_vec_aggregated[int(lane_group2[k4])];
                         //cout<<"x1_lane: "<<x1_lane<<endl;

                         //if ((x1_lane>1450) && (x1_lane<1550))
                         //{
                             //override_lane_group2=1;
                             override_lane_group1_mux=(mux_lane_vec_aggregated_previous[int(lane_group1_previous[k5])]+mux_lane_vec_aggregated[int(lane_group1[k4])])/2;
                             override_lane_group1_muy=(muy_lane_vec_aggregated_previous[int(lane_group1_previous[k5])]+muy_lane_vec_aggregated[int(lane_group1[k4])])/2;
                             override_lane_group1_base_ptx=(base_ptx_lane_vec_aggregated_previous[int(lane_group1_previous[k5])]+base_ptx_lane_vec_aggregated[int(lane_group1[k4])])/2;
                             override_lane_group1_base_pty=(base_pty_lane_vec_aggregated_previous[int(lane_group1_previous[k5])]+base_pty_lane_vec_aggregated[int(lane_group1[k4])])/2;
                             Lintersection=-override_lane_group1_base_pty/override_lane_group1_muy;
                             x1_lane=override_lane_group1_base_ptx+Lintersection*override_lane_group1_mux;
                             //cout<<"x1_lane b: "<<x1_lane<<endl;


                             if ((x1_lane>1300) && (x1_lane<1365))
                             {
                                 override_lane_group1=1;
                                 override_break=1;
                                 //cin>>any;
                                 break;
                             }

                             //cin>>any;
                         //}
                     }
                 }
                 }

                 if (override_break==1)
                 {
                     break;
                 }

             }    

         }       

             
         //mux_lane_vec_final1=mux_lane_vec_acc/count_lane_group1; 
         //muy_lane_vec_final1=muy_lane_vec_acc/count_lane_group1;
         //base_ptx_lane_vec_final1=base_ptx_lane_vec_acc/count_lane_group1;
         //base_pty_lane_vec_final1=base_pty_lane_vec_acc/count_lane_group1;

         if (count_marks_filtered_grp1>0)
         {
            mux_lane_vec_final1=mux_lane_vec_acc/count_marks_filtered_grp1; 
            muy_lane_vec_final1=muy_lane_vec_acc/count_marks_filtered_grp1;
            base_ptx_lane_vec_final1=base_ptx_lane_vec_acc/count_marks_filtered_grp1;
            base_pty_lane_vec_final1=base_pty_lane_vec_acc/count_marks_filtered_grp1;
         }


         

    }
            
    else
    {

        if (count_lane_group1==1)
        {


            if (override_lane_group1==0)
            {

             for (int k4=0 ; k4<count_lane_group1; k4++)
             {
                 override_break=0;
                 for (int k5=0; k5<count_lane_group1_previous; k5++)     
                 {
                 //if (k5!=k4)
                 //{
                     angleA=180*(atan(muy_lane_vec_aggregated[int(lane_group1[k4])]/mux_lane_vec_aggregated[int(lane_group1[k4])])/3.14159);
                     angleB=180*(atan(muy_lane_vec_aggregated_previous[int(lane_group1_previous[k5])]/mux_lane_vec_aggregated_previous[int(lane_group1_previous[k5])])/3.14159);
                     if ((abs(angleA-angleB)<6) && (30<angleA) && (angleA<45)) 
                     {
                         //cout<<"aligned"<<endl;
                         //cout<<"angle: "<<180*(atan(muy_lane_vec_aggregated[int(lane_group1[k4])]/mux_lane_vec_aggregated[int(lane_group1[k4])])/3.14159)<<endl;
                         //#intersecting with top of image        
                         //Lintersection=-base_pty_lane_vec_aggregated[int(lane_group2[k4])]/muy_lane_vec_aggregated[int(lane_group2[k4])];
                         //x1_lane=base_ptx_lane_vec_aggregated[int(lane_group2[k4])]+Lintersection*mux_lane_vec_aggregated[int(lane_group2[k4])];
                         //cout<<"x1_lane: "<<x1_lane<<endl;

                         //if ((x1_lane>1450) && (x1_lane<1550))
                         //{
                             //override_lane_group2=1;
                             override_lane_group1_mux=(mux_lane_vec_aggregated_previous[int(lane_group1_previous[k5])]+mux_lane_vec_aggregated[int(lane_group1[k4])])/2;
                             override_lane_group1_muy=(muy_lane_vec_aggregated_previous[int(lane_group1_previous[k5])]+muy_lane_vec_aggregated[int(lane_group1[k4])])/2;
                             override_lane_group1_base_ptx=(base_ptx_lane_vec_aggregated_previous[int(lane_group1_previous[k5])]+base_ptx_lane_vec_aggregated[int(lane_group1[k4])])/2;
                             override_lane_group1_base_pty=(base_pty_lane_vec_aggregated_previous[int(lane_group1_previous[k5])]+base_pty_lane_vec_aggregated[int(lane_group1[k4])])/2;
                             Lintersection=-override_lane_group1_base_pty/override_lane_group1_muy;
                             x1_lane=override_lane_group1_base_ptx+Lintersection*override_lane_group1_mux;
                             //cout<<"x1_lane c: "<<x1_lane<<endl;


                             if ((x1_lane>1300) && (x1_lane<1365))
                             {
                                 override_lane_group1=1;
                                 //cin>>any;
                                 override_break=1;
                                 break;
                             }

                             //cin>>any;
                         //}
                     }                   
                 //}
                     
                 }
                 if (override_break==1)
                 {
                     break;
                 }

              }    

            }
          
            //mux_lane_vec_final1=mux_lane_vec_aggregated[int(lane_group1[0])];
            //muy_lane_vec_final1=muy_lane_vec_aggregated[int(lane_group1[0])];
            //base_ptx_lane_vec_final1=base_ptx_lane_vec_aggregated[int(lane_group1[0])];
            //base_pty_lane_vec_final1=base_pty_lane_vec_aggregated[int(lane_group1[0])];

            //restricted filtering here
            Lintersection=-base_pty_lane_vec_aggregated[int(lane_group1[0])]/muy_lane_vec_aggregated[int(lane_group1[0])];
            x1_lane=base_ptx_lane_vec_aggregated[int(lane_group1[0])]+Lintersection*mux_lane_vec_aggregated[int(lane_group1[0])];

            if ((x1_lane>1300-15) && (x1_lane<1350+15))
            {

                mux_lane_vec_final1=mux_lane_vec_aggregated[int(lane_group1[0])];
                muy_lane_vec_final1=muy_lane_vec_aggregated[int(lane_group1[0])];
                base_ptx_lane_vec_final1=base_ptx_lane_vec_aggregated[int(lane_group1[0])];
                base_pty_lane_vec_final1=base_pty_lane_vec_aggregated[int(lane_group1[0])];

                //count_marks_filtered_grp1=count_marks_filtered_grp1+1;
                //cout<<"count_marks_filtered_grp1 b: "<<count_marks_filtered_grp1<<endl;
                //cin>>any;
            }

        }
    }
        
        
            
    if (count_lane_group2>=2)
    {
        
         mux_lane_vec_acc=0;
         muy_lane_vec_acc=0;
         base_ptx_lane_vec_acc=0;
         base_pty_lane_vec_acc=0;
        
         for (int k4=0 ; k4<count_lane_group2; k4++)
         {
                

             //restricted filtering here
             Lintersection=-base_pty_lane_vec_aggregated[int(lane_group2[k4])]/muy_lane_vec_aggregated[int(lane_group2[k4])];
             x1_lane=base_ptx_lane_vec_aggregated[int(lane_group2[k4])]+Lintersection*mux_lane_vec_aggregated[int(lane_group2[k4])];

             if ((x1_lane>1460-15) && (x1_lane<1510+15))
             {

                 mux_lane_vec_acc=mux_lane_vec_acc+mux_lane_vec_aggregated[int(lane_group2[k4])];
                 muy_lane_vec_acc=muy_lane_vec_acc+muy_lane_vec_aggregated[int(lane_group2[k4])];
                 base_ptx_lane_vec_acc=base_ptx_lane_vec_acc+base_ptx_lane_vec_aggregated[int(lane_group2[k4])];
                 base_pty_lane_vec_acc=base_pty_lane_vec_acc+base_pty_lane_vec_aggregated[int(lane_group2[k4])];
                 count_marks_filtered_grp2=count_marks_filtered_grp2+1;
                 cout<<"considered"<<endl;

             }

             override_break=0;
             for (int k5=0; k5<count_lane_group2; k5++)
             {
                 if (k5!=k4)
                 {
                     angleA=180*(atan(muy_lane_vec_aggregated[int(lane_group2[k4])]/mux_lane_vec_aggregated[int(lane_group2[k4])])/3.14159);
                     angleB=180*(atan(muy_lane_vec_aggregated[int(lane_group2[k5])]/mux_lane_vec_aggregated[int(lane_group2[k5])])/3.14159);
                     if ((abs(angleA-angleB)<6) && (-20>angleA) && (angleA>-40))
                     {
                         //cout<<"aligned"<<endl;
                         //cout<<"angle: "<<180*(atan(muy_lane_vec_aggregated[int(lane_group2[k4])]/mux_lane_vec_aggregated[int(lane_group2[k4])])/3.14159)<<endl;
                         //#intersecting with top of image        
                         //Lintersection=-base_pty_lane_vec_aggregated[int(lane_group2[k4])]/muy_lane_vec_aggregated[int(lane_group2[k4])];
                         //x1_lane=base_ptx_lane_vec_aggregated[int(lane_group2[k4])]+Lintersection*mux_lane_vec_aggregated[int(lane_group2[k4])];
                         //cout<<"x1_lane: "<<x1_lane<<endl;

                         //if ((x1_lane>1450) && (x1_lane<1550))
                         //{
                             //override_lane_group2=1;
                             override_lane_group2_mux=(mux_lane_vec_aggregated[int(lane_group2[k5])]+mux_lane_vec_aggregated[int(lane_group2[k4])])/2;
                             override_lane_group2_muy=(muy_lane_vec_aggregated[int(lane_group2[k5])]+muy_lane_vec_aggregated[int(lane_group2[k4])])/2;
                             override_lane_group2_base_ptx=(base_ptx_lane_vec_aggregated[int(lane_group2[k5])]+base_ptx_lane_vec_aggregated[int(lane_group2[k4])])/2;
                             override_lane_group2_base_pty=(base_pty_lane_vec_aggregated[int(lane_group2[k5])]+base_pty_lane_vec_aggregated[int(lane_group2[k4])])/2;
                             Lintersection=-override_lane_group2_base_pty/override_lane_group2_muy;
                             x1_lane=override_lane_group2_base_ptx+Lintersection*override_lane_group2_mux;
                             //cout<<"x1_lane a: "<<x1_lane<<endl;

                             if ((x1_lane>1460) && (x1_lane<1540))
                             {
                                 override_lane_group2=1;
                                 override_break=1;
                                 break;
                             }

                             //cin>>any;
                         //}
                     }

                 }
             }
        
             if (override_break==1)
             {
                 break;
             }

         }


         if (override_lane_group2==0)
         {

             for (int k4=0 ; k4<count_lane_group2; k4++)
             {
                 override_break=0;
                 for (int k5=0; k5<count_lane_group2_previous; k5++)     
                 {
                 if (k5!=k4)
                 {
                     angleA=180*(atan(muy_lane_vec_aggregated[int(lane_group2[k4])]/mux_lane_vec_aggregated[int(lane_group2[k4])])/3.14159);
                     angleB=180*(atan(muy_lane_vec_aggregated_previous[int(lane_group2_previous[k5])]/mux_lane_vec_aggregated_previous[int(lane_group2_previous[k5])])/3.14159);
                     if ((abs(angleA-angleB)<6) && (-20>angleA) && (angleA>-40))
                     {
                         //cout<<"aligned"<<endl;
                         //cout<<"angle: "<<180*(atan(muy_lane_vec_aggregated[int(lane_group2[k4])]/mux_lane_vec_aggregated[int(lane_group2[k4])])/3.14159)<<endl;
                         //#intersecting with top of image        
                         //Lintersection=-base_pty_lane_vec_aggregated[int(lane_group2[k4])]/muy_lane_vec_aggregated[int(lane_group2[k4])];
                         //x1_lane=base_ptx_lane_vec_aggregated[int(lane_group2[k4])]+Lintersection*mux_lane_vec_aggregated[int(lane_group2[k4])];
                         //cout<<"x1_lane: "<<x1_lane<<endl;

                         //if ((x1_lane>1450) && (x1_lane<1550))
                         //{
                             //override_lane_group2=1;
                             override_lane_group2_mux=(mux_lane_vec_aggregated_previous[int(lane_group2_previous[k5])]+mux_lane_vec_aggregated[int(lane_group2[k4])])/2;
                             override_lane_group2_muy=(muy_lane_vec_aggregated_previous[int(lane_group2_previous[k5])]+muy_lane_vec_aggregated[int(lane_group2[k4])])/2;
                             override_lane_group2_base_ptx=(base_ptx_lane_vec_aggregated_previous[int(lane_group2_previous[k5])]+base_ptx_lane_vec_aggregated[int(lane_group2[k4])])/2;
                             override_lane_group2_base_pty=(base_pty_lane_vec_aggregated_previous[int(lane_group2_previous[k5])]+base_pty_lane_vec_aggregated[int(lane_group2[k4])])/2;
                             Lintersection=-override_lane_group2_base_pty/override_lane_group2_muy;
                             x1_lane=override_lane_group2_base_ptx+Lintersection*override_lane_group2_mux;
                             //cout<<"x1_lane b: "<<x1_lane<<endl;


                             if ((x1_lane>1460) && (x1_lane<1540))
                             {
                                 override_lane_group2=1;
                                 override_break=1;
                                 break;
                             }

                             //cin>>any;
                         //}
                     }
                 }
                 }

                 if (override_break==1)
                 {
                     break;
                 }

             }    

         }         
    
         //mux_lane_vec_final2=mux_lane_vec_acc/count_lane_group2; 
         //muy_lane_vec_final2=muy_lane_vec_acc/count_lane_group2;
         //base_ptx_lane_vec_final2=base_ptx_lane_vec_acc/count_lane_group2;
         //base_pty_lane_vec_final2=base_pty_lane_vec_acc/count_lane_group2;

         if (count_marks_filtered_grp2>0)
         {
             mux_lane_vec_final2=mux_lane_vec_acc/count_marks_filtered_grp2; 
             muy_lane_vec_final2=muy_lane_vec_acc/count_marks_filtered_grp2;
             base_ptx_lane_vec_final2=base_ptx_lane_vec_acc/count_marks_filtered_grp2;
             base_pty_lane_vec_final2=base_pty_lane_vec_acc/count_marks_filtered_grp2;
         }

    }
            
    else
    {

        if (count_lane_group2==1)
        {
            
            if (override_lane_group2==0)
            {

             for (int k4=0 ; k4<count_lane_group2; k4++)
             {
                 override_break=0;
                 for (int k5=0; k5<count_lane_group2_previous; k5++)     
                 {
                 //if (k5!=k4)
                 //{
                     angleA=180*(atan(muy_lane_vec_aggregated[int(lane_group2[k4])]/mux_lane_vec_aggregated[int(lane_group2[k4])])/3.14159);
                     angleB=180*(atan(muy_lane_vec_aggregated_previous[int(lane_group2_previous[k5])]/mux_lane_vec_aggregated_previous[int(lane_group2_previous[k5])])/3.14159);
                     if ((abs(angleA-angleB)<6) && (-20>angleA) && (angleA>-40)) 
                     {
                         //cout<<"aligned"<<endl;
                         //cout<<"angle: "<<180*(atan(muy_lane_vec_aggregated[int(lane_group2[k4])]/mux_lane_vec_aggregated[int(lane_group2[k4])])/3.14159)<<endl;
                         //#intersecting with top of image        
                         //Lintersection=-base_pty_lane_vec_aggregated[int(lane_group2[k4])]/muy_lane_vec_aggregated[int(lane_group2[k4])];
                         //x1_lane=base_ptx_lane_vec_aggregated[int(lane_group2[k4])]+Lintersection*mux_lane_vec_aggregated[int(lane_group2[k4])];
                         //cout<<"x1_lane: "<<x1_lane<<endl;

                         //if ((x1_lane>1450) && (x1_lane<1550))
                         //{
                             //override_lane_group2=1;
                             override_lane_group2_mux=(mux_lane_vec_aggregated_previous[int(lane_group2_previous[k5])]+mux_lane_vec_aggregated[int(lane_group2[k4])])/2;
                             override_lane_group2_muy=(muy_lane_vec_aggregated_previous[int(lane_group2_previous[k5])]+muy_lane_vec_aggregated[int(lane_group2[k4])])/2;
                             override_lane_group2_base_ptx=(base_ptx_lane_vec_aggregated_previous[int(lane_group2_previous[k5])]+base_ptx_lane_vec_aggregated[int(lane_group2[k4])])/2;
                             override_lane_group2_base_pty=(base_pty_lane_vec_aggregated_previous[int(lane_group2_previous[k5])]+base_pty_lane_vec_aggregated[int(lane_group2[k4])])/2;
                             Lintersection=-override_lane_group2_base_pty/override_lane_group2_muy;
                             x1_lane=override_lane_group2_base_ptx+Lintersection*override_lane_group2_mux;
                             //cout<<"x1_lane c: "<<x1_lane<<endl;


                             if ((x1_lane>1460) && (x1_lane<1540))
                             {
                                 override_lane_group2=1;
                                 override_break=1;
                                 break;
                             }

                             //cin>>any;
                         //}
                     }
                 //}
                 }

                 if (override_break==1)
                 {
                     break;
                 }

              }    

             }                     


            //restricted filtering here
            Lintersection=-base_pty_lane_vec_aggregated[int(lane_group2[0])]/muy_lane_vec_aggregated[int(lane_group2[0])];
            x1_lane=base_ptx_lane_vec_aggregated[int(lane_group2[0])]+Lintersection*mux_lane_vec_aggregated[int(lane_group2[0])];

            if ((x1_lane>1460-15) && (x1_lane<1510+15))
            {

                mux_lane_vec_final2=mux_lane_vec_aggregated[int(lane_group2[0])];
                muy_lane_vec_final2=muy_lane_vec_aggregated[int(lane_group2[0])];
                base_ptx_lane_vec_final2=base_ptx_lane_vec_aggregated[int(lane_group2[0])];
                base_pty_lane_vec_final2=base_pty_lane_vec_aggregated[int(lane_group2[0])];

                count_marks_filtered_grp2=count_marks_filtered_grp2+1;
                cout<<"considered"<<endl;
            }

        }

    }

    return;         

}
