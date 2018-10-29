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

using namespace std;
using namespace cv;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// TRACKER FUNCTION
//
// The function below implements the method to track a vehicle trough importance sampling (particle filtering). The procedure starts by generating 10 random particles (current cloud) around a
// previously determined initial cloud of particles that corresponds to the previous frame. Then an evaluation is performed to determine the closest bounding box in distance to the current particle cloud.
// Once we find the closest vehicle bounding box its centroid is used to perform the update part of importance sampling. Thus an updated distribution is generated based on the influence of the new
// information provided by the found bounding box. A new cloud of particles is generated based on the updated distribution. The new cloud captures the state of current vehicle position as the best estimate
// given previous and current information and the new cloud is expected to be used as an initial cloud for the next image frame tracking processing.
//
// Two control methods are implemented in this function. One method provides a mechanism to use the cloud of particles to preserve vehicle information positioning when the bounding box detection for
// such vehicle dissapears. When the boudning box re-appears again the cloud information allows the proper identification without mistakingly assigning a different bounding box. The second control method
// allows to avoid taking a bounding box that was already assigned to another tracker even if there was no bounding box dissapearance (this avoids the problem of tracker merging). These control
// methods are governed by the heuristics of vehicle trajetory.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void tracker(vector< vector<double>> &x_particle_vehicles,vector< vector<double>> &y_particle_vehicles,vector<double> &delta_x_vehicles,vector<double> &delta_y_vehicles,int Number_of_objects,vector<double>  &centroid_x_previous, vector<double> &centroid_y_previous,vector<int> &initialize_vehicles,vector<int> &count_holding_vehicles,vector< vector<double>> &distance_to_particle_identified_previous_input,int trackerID,int count_tracked_vehicles,vector<double> &cx_tracked,vector<double> &cy_tracked,Mat &frame2,vector< vector<int>> d)
{

      /////////////////////////////////////////////////////////////////////////////////////////////////
      //Initialization of Variables
      vector<double> x_particle_out(10,0);
      vector<double> y_particle_out(10,0);
      vector<double> x_particle_next(10,0);
      vector<double> y_particle_next(10,0);
      vector<double> distance_to_particle(10,0);
      vector<double> distance_to_particle_identified(10,0);

      vector<double> w(10,0);     //weights to be used for the resampling part of Importance Sampling
      vector<double> cdf(10,0);   //vector to be used for the cumulative distribution function

      vector<double> x_particle_resampled(10,0);
      vector<double> y_particle_resampled(10,0);

      vector<double> mean(2,0);
      vector< vector<double>> cov(2, vector<double> (2,0));
      vector< vector<double>> cholesky(2, vector<double> (2,0));
      vector<double> random_vect(2,0);
      default_random_engine generator(time(0));
      normal_distribution<double> distribution(0,1);

      vector<int> indexes_sorted(10,0);

      double cx;
      double cy;
      int x1;
      int y1;
      int x2;
      int y2;
      double min_average_distance;
      double acum_distance_particles;
      int identified;
      int cx_identified;
      int cy_identified;
      double average_distance_to_particle;
      double accum;
      double sum_cdf;
      double draw_uniform;
      double delta_x_out;
      double delta_y_out;
      int initialization_delta;
      int merging_conflict;



      /////////////////////////////////////////////////////////////////////////////////////////////////////////
      //I) GENERATION OF 10 RANDOM PARTICLES

      for (int k1=0; k1<10; k1++)
      {

          mean[0]=x_particle_vehicles[trackerID][k1]+delta_x_vehicles[trackerID];
          mean[1]=y_particle_vehicles[trackerID][k1]+delta_y_vehicles[trackerID];

          cov[0][0]=10;
          cov[0][1]=0;
          cov[1][0]=0;
          cov[1][1]=10;


          //simple cholesky factorization since the random variables corresponding to the x component
          //and to the y component of the particle are uncorrelated (this holds in general). The x and y
          //components are assumed distributed as bivariate normal.

          cholesky[0][0]=pow(cov[0][0],0.5);
          cholesky[0][1]=0;
          cholesky[1][0]=0;
          cholesky[1][1]=pow(cov[1][1],0.5);

          random_vect[0]=distribution(generator);
          random_vect[1]=distribution(generator);

          x_particle_next[k1] = mean[0] + cholesky[0][0]*random_vect[0]+cholesky[0][1]*random_vect[1];
          y_particle_next[k1] = mean[1] + cholesky[1][0]*random_vect[0]+cholesky[1][1]*random_vect[1];


          circle(frame2,Point(int(x_particle_next[k1]), int(y_particle_next[k1])), 8, Scalar(255,0,0), -1);

      }



      //////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //II) IDENTIFYING THE BOUNDING BOX THROUGH THE PARTICLES: WHICH BOUNDING BOX CORRESPONDS TO THESE PARTICLES

      min_average_distance = 10000;

      for (int k = 0; k<Number_of_objects; k++)
      {
          x1=d[k][1];
          y1=d[k][0];
          x2=d[k][3];
          y2=d[k][2];


          //centroid of the bounding box
          cx = (x1+x2)/2;
          cy = (y1+y2)/2;

          //determine the closest bounding box to the cloud of particles
          acum_distance_particles = 0;

          for (int k1 = 0; k1 < 10; k1++)
          {
              distance_to_particle[k1]=pow(pow(cx-x_particle_next[k1],2)+pow(cy-y_particle_next[k1],2),0.5);
              acum_distance_particles=acum_distance_particles+distance_to_particle[k1];
          }

          average_distance_to_particle = acum_distance_particles/10;

          if (average_distance_to_particle < min_average_distance)
          {
              min_average_distance = average_distance_to_particle;
              identified = k;
              cx_identified = cx; //cx and cy: indentified the centroid of bounding box corresponding to this cloud of particles
              cy_identified = cy;

              for (int j1=0; j1 < 10; j1++)
              {
                  distance_to_particle_identified[j1]=distance_to_particle[j1];
              }
          }


      }




      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //III) TRACKER CONTROL I: PROCESS TO RESOLVE BOUNDING BOX DISSAPEARANCE

      if  (count_holding_vehicles[trackerID]==0)
      {
          if (abs(cx_identified-centroid_x_previous[trackerID]) > 100) //the bounding box possibly dissapeared
          {
              count_holding_vehicles[trackerID]=count_holding_vehicles[trackerID]+1;
              cx_identified=centroid_x_previous[trackerID];
              cy_identified=centroid_y_previous[trackerID];
              for (int j1=0; j1 < 10; j1++)
              {
                  distance_to_particle_identified[j1]=distance_to_particle_identified_previous_input[trackerID][j1];
              }
          }

          else
          {
              for( int k4=1; k4 < count_tracked_vehicles+1; k4++)
              {
                  if (k4 != trackerID)
                  {
                      if ( (pow(pow(cx_identified-cx_tracked[k4],2)+pow(cy_identified-cy_tracked[k4],2),0.5) < 50) && (count_holding_vehicles[k4] == 0) ) //due to bounding box dissapearance another bounding box corresponding to a different vehicle (another trackerID) may have been assigned to the current vehicle
                      {
                          count_holding_vehicles[trackerID]=count_holding_vehicles[trackerID]+1;

                          //override current identification of particles (in other words override current bounding box assigment) and use data from previous successful bounding box assigment while we hold waiting for the bounding box to reappear
                          cx_identified = centroid_x_previous[trackerID];
                          cy_identified = centroid_y_previous[trackerID];

                          for (int j1=0; j1 < 10; j1++)
                          {
                              distance_to_particle_identified[j1]=distance_to_particle_identified_previous_input[trackerID][j1];
                              //delta_x_holding=delta_x_input
                          }

                          break;
                      }
                  }
              }
          }

      }

      else   //we are in a process of holding for the bounding box to appear
      {
          if (count_holding_vehicles[trackerID] >= 4) //if bounding box doesn't appear after 4 frames then shutdown the current tracker assigned to the box
          {
              count_holding_vehicles[trackerID] = 0;
              initialize_vehicles[trackerID] = 0; //this tracker will go through a brand new assignment process
          }

          else   //we are waiting for the bounding box to appear
          {
              if (abs(cx_identified-centroid_x_previous[trackerID]) > 100)
              {
                   count_holding_vehicles[trackerID] = count_holding_vehicles[trackerID] + 1;
                   cx_identified = centroid_x_previous[trackerID];
                   cy_identified = centroid_y_previous[trackerID];
                   for (int j1=0; j1 < 10; j1++)
                   {
                       distance_to_particle_identified[j1] = distance_to_particle_identified_previous_input[trackerID][j1];
                   }

              }

              else //the bounding box corresponding to this tracker may have reappeared
              {
                   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                   //IV) TRACKER CONTROL II: RESOLVING CONFLICT ON REASSIGNMENT OF BOUNDING BOX AFTER DISSAPEARNCE DUE TO CANDIDATE BOUNDING BOX BELONGING TO ANTHER TRACKER

                   merging_conflict = 0;
                   for ( int k4 = 1; k4 < count_tracked_vehicles+1; k4++)
                   {
                       if (k4 != trackerID)
                       {
                           if (pow(pow(cx_identified-cx_tracked[k4],2)+pow(cy_identified-cy_tracked[k4],2),0.5) < 50) //there is conflict with another tracker, bounding box assignment is not possible so we keep holding
                           {
                               count_holding_vehicles[trackerID] = count_holding_vehicles[trackerID] + 1;
                               cx_identified=centroid_x_previous[trackerID];
                               cy_identified=centroid_y_previous[trackerID];
                               for (int j1=0; j1 < 10; j1++)
                               {
                                   distance_to_particle_identified[j1] = distance_to_particle_identified_previous_input[trackerID][j1];
                               }
                              merging_conflict = 1;
                              break;
                           }
                       }
                   }

                   if  (merging_conflict == 0)
                   {
                       count_holding_vehicles[trackerID] = 0; //the bounding box reappeared
                   }
              }
          }

      }


      rectangle(frame2, Point(int(d[identified][1]),int(d[identified][0])), Point(int(d[identified][3]),int(d[identified][2])), Scalar(0,0,255), 2, 1);
      putText(frame2,to_string(trackerID),Point(int(cx_identified),int(cy_identified)), FONT_HERSHEY_PLAIN, 2,Scalar(0,255,0),2);

      cout<<"tracker"<<endl;

      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //V) IMPORTANCE SAMPLING PROCESSING TO UPDATE DISTRIBUTION TO BE USED TO GENERATE THE NEXT CLOUD OF PARTICLES FOR THE NEXT FRAME

      //reindexing
      vector<pair<double,double> > vector_pairs;

      for (int k2=0; k2<10; k2++)
      {
          vector_pairs.push_back( make_pair(distance_to_particle_identified[k2],k2) );
      }

      sort(vector_pairs.begin(),vector_pairs.end());

      for (int k2=0; k2<10; k2++)
      {
          indexes_sorted[k2] = vector_pairs[k2].second;
      }

      //weights
      accum=0;
      for (int i=0; i < 10; i++)
      {
          w[i]=exp(-0.5*(pow(distance_to_particle_identified[indexes_sorted[i]],0.5)));
          accum=accum+w[i];
      }

      //cumulative distribution
      sum_cdf=0;
      for (int i=0; i < 10; i++)
      {
          sum_cdf=sum_cdf+w[i]/accum;
          cdf[i]=sum_cdf;
      }


      //uniform: use prescribed distribution function based on cumulative distribution function previously built
      //This way the particles are drawn from a distribution induced by the weights
      for (int i=0; i < 10; i++)
      {

          draw_uniform = ((double)rand()/RAND_MAX);

          for (int j=0; j < 10; j++)
          {
              if (draw_uniform <= cdf[j])
              {
                  x_particle_resampled[i]=x_particle_next[indexes_sorted[j]];
                  y_particle_resampled[i]=y_particle_next[indexes_sorted[j]];

                  break;
              }
          }
      }

      for (int k1=0; k1 < 10; k1++)
      {
          x_particle_vehicles[trackerID][k1] = x_particle_resampled[k1];
          y_particle_vehicles[trackerID][k1] = y_particle_resampled[k1];
      }


      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //VI) FINAL PROCESSINGS FOR THIS CURRENT FRAME ON THE CURRENT TRACKER, AND PREPARATION FOR NEXT FRAME

      if (initialize_vehicles[trackerID] == 0) //This tracker is being shutdown and it will go through a new vehicle assignment process
      {
          delta_x_vehicles[trackerID] = 0;
          delta_y_vehicles[trackerID] = 0;
          cx_tracked[trackerID]=0;
          cy_tracked[trackerID]=0;
      }
      else
      {

          initialization_delta = 1;
          delta_x_vehicles[trackerID]=cx_identified-centroid_x_previous[trackerID];
          delta_y_vehicles[trackerID]=cy_identified-centroid_y_previous[trackerID];
          centroid_x_previous[trackerID]=cx_identified;
          centroid_y_previous[trackerID]=cy_identified;
          cx_tracked[trackerID]=cx_identified;
          cy_tracked[trackerID]=cy_identified;

          for (int j1=0; j1 < 10; j1++)
          {
              distance_to_particle_identified_previous_input[trackerID][j1] = distance_to_particle_identified[j1];
          }

      }


}


//////////////////////////////////// MAIN TRACKING ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// The procedure below provides the main steps necessary to provide continuous tracking of vehicles through several image frames.
// The first part of the procedure loads pre-recorded bounding-box detections recorded as coordinate values on a text file. The
// Final implementation of this tracker assumes that this step will be substituted by a procedure that will provide the information coming
// from the actual vehicle detector being used.
//
// The second step is tracker initialization which is performed taking into account bounding boxes previously assigned to other trackers so
// that only unassigned bounding boxes are available for every new tracker. In this context if a previously assgined tracker is going through a
// problematic situation/event such as dealing with the dissapearance of its assigned bounding box, then the initialization attempts to avoid
// the bounding box associated with the problematic situation. It could happen that the tracker seeking initialization could "steal" the bounding
// box from a tracker which is experiencing problems. This helps to avoid a "racing" condition between trackers.
//
// For trackers succesully initialized the third step invokes the Tracker function which delivers tracking procedures for the current frame.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

main(){

      ////////////////////////////////////////////////
      // Variable Initialization

      ifstream file;
      int data_value;
      char character[20];
      int input;
      int Lc;
      int dmatrix[4000][30][4];
      int Lc_vec[4000];
      //int d[4000][4];
      vector< vector<int>> d(4000, vector<int> (4,0));
      int index_row;
      int index;
      int index_detections;

      vector<int> initialize_vehicles(20,0);
      vector<double> mean(2,0);
      vector< vector<double>> cov(2, vector<double> (2,0));
      vector< vector<double>> cholesky(2, vector<double> (2,0));
      vector<double> random_vect(2,0);
      vector< vector<double>> x_particle_vehicles(20, vector<double> (10,0));
      vector< vector<double>> y_particle_vehicles(20, vector<double> (10,0));
      vector<double> delta_x_vehicles(20,0);
      vector<double> delta_y_vehicles(20,0);
      vector<double> cx_previous_vehicles(20,0);
      vector<double> cy_previous_vehicles(20,0);
      vector<double> cx_tracked(20,0);
      vector<double> cy_tracked(20,0);
      vector<int> count_holding_vehicles(20,0);
      vector< vector<double>> distance_to_particle_identified_previous_vehicles(20, vector<double> (10,0));

      int vehicle_found_for_initialization;
      int vehicle_index_for_initialization;
      int count_tracked_vehicles;

      int x1;
      int y1;
      int x2;
      int y2;
      double cx;
      double cy;
      int conflict;

      default_random_engine generator(time(0));  //randomizing the seed
      normal_distribution<double> distribution(0,1);
      string file_name;





      /////////////////////////////////////////////////////////////////////////////
      // LOADING OF BOUNDING BOX DETECTIONS (FROM A PRE-RECORDED FILE)

      index_row=0;

      //Loading detected Bounding Boxes information

      file.open("vehicle_coordinates.txt");

      if (file.is_open())
      {
          while(file.good())
          {

              //Loading number of vehicles
              file.getline(character, 256,' ');
              Lc = atof(character);
              Lc_vec[index_row] = Lc;

              //Loading 4 coordinates per vehicle
              for (int i=0;i<Lc;i++)
              {
                  for (int j=0;j<4;j++)
                  {
                      file.getline(character, 256,' ');
                      data_value = atof(character);
                      dmatrix[index_row][i][j] = data_value;
                      //printf("data: %d ",data_value);
                  }
              }

              index_row = index_row+1;

              //cin>>input;
          }
      }



      ////////////////////////////////////////////////////////////////////////////////////////
      // CONTINUOUS TRACKING CYCLE
      ////////////////////////////////////////////////////////////////////////////////////////

      index=1280;
      index_detections = 0;
      count_tracked_vehicles = 8;



      while (1)
      {

          ////////////////////////////////////////////////////////////////////////////////////
          //I) LOADING IMAGE FRAME
          file_name = "./Hwy101_frames/"+to_string(index)+".jpg";

          Mat frame = imread(file_name.c_str(), IMREAD_GRAYSCALE);
          Mat frame2 = imread(file_name.c_str());


          Lc = Lc_vec[index_detections];


          for (int k = 0 ; k < Lc ; k++)
          {
              d[k][0] = dmatrix[index_detections][k][0];
              d[k][1] = dmatrix[index_detections][k][1];
              d[k][2] = dmatrix[index_detections][k][2];
              d[k][3] = dmatrix[index_detections][k][3];


              rectangle(frame2,Point(d[k][1],d[k][0]),Point(d[k][3],d[k][2]),Scalar(0,0,0),2,1);
          }


          ////////////////////////////////////////////////////////////////////////////////////////
          //II) TRACKER INITIALIZATION

          for (int veh = 1; veh <= count_tracked_vehicles; veh++)
          {
              if (initialize_vehicles[veh] == 0)
              {
                  vehicle_found_for_initialization = 0;

                  for (int k2 = 0; k2 < Lc; k2++)
                  {

                      x1 = d[k2][1];
                      y1 = d[k2][0];
                      x2 = d[k2][3];
                      y2 = d[k2][2];

                      cx = (x1+x2)/2;
                      cy = (y1+y2)/2;

                      conflict=0;

                      //Evaluating if the current bounding box was already assigned to another tracker
                      for (int k3 = 1; k3 < count_tracked_vehicles + 1; k3++)
                      {

                          if (pow(pow(cx-cx_tracked[k3],2)+pow(cy-cy_tracked[k3],2),0.5) < 50)
                          {
                              conflict=1;
                              break;
                          }
                          else
                          {
                              if ((count_holding_vehicles[k3] > 0) || (cy_tracked[k3] > 1600)) //if the vehicle(a.k.a bounding box) being probed is holding then we need to be more restrictive because the holding is being done with cx_previous and cy_previous which is farther from current bounding box that in principle corresponds to holding vehicle
                              {
                                  if (pow(pow(cx-cx_tracked[k3],2)+pow(cy-cy_tracked[k3],2),0.5) < 100)
                                  {
                                     conflict=1;
                                     break;
                                  }
                              }
                          }


                      }

                      if (conflict == 0)
                      {
                          cx_tracked[veh]=cx;
                          cy_tracked[veh]=cy;
                          vehicle_found_for_initialization=1;  //succesful initialization of this tracker
                          vehicle_index_for_initialization=k2;
                          break;
                      }


                  }


                  // Tracker has been initialized. Then generating initial cloud of particles (proposed distribution)
                  if ( vehicle_found_for_initialization == 1)
                  {
                      mean[0]=cx;
                      mean[1]=cy;

                      cov[0][0]=10;
                      cov[0][1]=0;
                      cov[1][0]=0;
                      cov[1][1]=10;

                      cx_previous_vehicles[veh]=cx;
                      cy_previous_vehicles[veh]=cy;



                      // Generation of 10 random particles
                      //
                      // The procedure implemented below provides the generation of a random vector with two components (x and y).
                      // The procedure is based on simple cholesky factorization since the individual random variables corresponding
                      // to the x component and to the y component of the particle are uncorrelated (this holds in general). The x and y
                      // components are assumed distributed as bivariate normal.

                      cholesky[0][0]=pow(cov[0][0],0.5);
                      cholesky[0][1]=0;
                      cholesky[1][0]=0;
                      cholesky[1][1]=pow(cov[1][1],0.5);


                      for (int k1=0; k1<10; k1++)
                      {

                          random_vect[0]=distribution(generator);
                          random_vect[1]=distribution(generator);
                          x_particle_vehicles[veh][k1] = mean[0] + cholesky[0][0]*random_vect[0]+cholesky[0][1]*random_vect[1];
                          y_particle_vehicles[veh][k1] = mean[1] + cholesky[1][0]*random_vect[0]+cholesky[1][1]*random_vect[1];

                          circle(frame2,Point(int(x_particle_vehicles[veh][k1]), int(y_particle_vehicles[veh][k1])), 8, Scalar(255,0,0), -1);

                      }

                      rectangle(frame2,Point(d[vehicle_index_for_initialization][1],d[vehicle_index_for_initialization][0]),Point(d[vehicle_index_for_initialization][3],d[vehicle_index_for_initialization][2]),Scalar(0,0,0),2,1);

                      initialize_vehicles[veh] = 1; //this tracker (a.k.a veh) has been initialized

                  }

              }

              //////////////////////////////////////////////////////////////////////////////////////////////////////////
              // v) PERFORM TRACKING ON CURRENT FRAME (FOR A TRACKER THAT HAS BEEN ALREADY INITIALIZED)
              else
              {
                  tracker(x_particle_vehicles,y_particle_vehicles,delta_x_vehicles,delta_y_vehicles,Lc,cx_previous_vehicles,cy_previous_vehicles,initialize_vehicles,count_holding_vehicles,distance_to_particle_identified_previous_vehicles,veh,count_tracked_vehicles,cx_tracked,cy_tracked,frame2,d);

              }

          }



          ///////////////////////////////////////////////////////////////////////////////
          // IV) DISPLAY
          namedWindow("frame", WINDOW_NORMAL);
          imshow("frame",frame2);
          waitKey(1);

          index = index + 1;
          index_detections = index_detections + 1;

      }

}
