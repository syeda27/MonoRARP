self.centroid_y_preself.deltaself.cdfimport cv2
import math
import numpy as np
import pickle

# JUAN CARLOS' version
# edited by Anjali

class ParticleTracker:
    def __init__(n_p, n_v):
        self.n_p = n_p
        self.n_v = n_v
        self.x_particle_out=np.zeros(n_p)
        self.y_particle_out=np.zeros(n_p)
        self.x_particle_next=np.zeros(n_p)
        self.y_particle_next=np.zeros(n_p)
        self.distance_to_particle=np.zeros(n_p)
        self.distance_to_particle_identified=np.zeros(n_p)

        self.w=np.zeros(n_p)     #weights to be used for the resampling part of Importance Sampling
        self.cdf=np.zeros(n_p)   #vector to be used for the cumulative distribution function

        self.x_particle_resampled=np.zeros(n_p)
        self.y_particle_resampled=np.zeros(n_p)
        self.initialize_vehicles=np.zeros(n_v)
        self.x_particle=np.zeros(n_v)
        self.y_particle=np.zeros(n_v)
        self.distance_to_particle_identified_previous=np.zeros(n_v)
        self.distance_to_particle_identified_previous_vehicles=np.zeros((n_v,n_v)) #what does this do?

        self.delta_x=0
        self.delta_y=0
        self.delta_x_vehicles=np.zeros(n_v)
        self.delta_y_vehicles=np.zeros(n_v)

        self.count_holding_vehicles=np.zeros(n_v)
        self.count_tracked_vehicles=8 #We set the number of trackers that will work simultaneously
        self.cx_tracked=np.zeros(20)
        self.cy_tracked=np.zeros(20)
        self.cx_previous_vehicles=np.zeros(n_v)
        self.cy_previous_vehicles=np.zeros(n_v)

        # where are these even defined in original code???
        self.count_holding_input = None
        self.centroid_x_previous = None
        self.centroid_y_previous = None

        # to be defined in create
        self.img = None
        self.box_info = None
        self.index = None
        self.initialize_flag = None
        self.d = None

    def create(box_info, image):
        self.box_info = box_info
        self.img = image

    def display_trackers(trackerID, identified):
        if (trackerID==1 or trackerID==2) and self.initialize_flag==1 and self.count_holding_input==0:
            cv2.rectangle(self.img, (self.d[identified][1],self.d[identified][0]), (self.d[identified][3],self.d[identified][2]), (0,0,255), 2, 1)
            cv2.putText(self.img,str(trackerID),(int(cx_identified),int(cy_identified)), cv2.FONT_HERSHEY_SIMPLEX, 2,(0,255,0),2)
        if (trackerID==3 or trackerID==4) and self.initialize_flag==1 and self.count_holding_input==0:
            cv2.rectangle(self.img, (self.d[identified][1],self.d[identified][0]), (self.d[identified][3],self.d[identified][2]), (0,255,0), 2, 1)
            cv2.putText(self.img,str(trackerID),(int(cx_identified),int(cy_identified)), cv2.FONT_HERSHEY_SIMPLEX, 2,(0,255,0),2)
        if (trackerID==5 or trackerID==6) and self.initialize_flag==1 and self.count_holding_input==0:
            cv2.rectangle(self.img, (self.d[identified][1],self.d[identified][0]), (self.d[identified][3],self.d[identified][2]), (255,0,0), 2, 1)
            cv2.putText(self.img,str(trackerID),(int(cx_identified),int(cy_identified)), cv2.FONT_HERSHEY_SIMPLEX, 2,(0,255,0),2)
        if (trackerID==7 or trackerID==8) and self.initialize_flag==1 and self.count_holding_input==0:
            cv2.rectangle(self.img, (self.d[identified][1],self.d[identified][0]), (self.d[identified][3],self.d[identified][2]), (150,100,0), 2, 1)
            cv2.putText(self.img,str(trackerID),(int(cx_identified),int(cy_identified)), cv2.FONT_HERSHEY_SIMPLEX, 2,(0,255,0),2)

    def create_cdf():
        #reindexing
        indexes_sorted=[b[0] for b in sorted(enumerate(self.distance_to_particle_identified),key=lambda i:i[1])]
        #weights
        accum=0
        for i in range(0,self.n_p):
            self.w[i]=math.exp(-0.5*((self.distance_to_particle_identified[indexes_sorted[i]])**0.5))
            accum=accum+self.w[i]
        #cumulative distribution
        sum_cdf=0
        for i in range(0,10):
            sum_cdf=sum_cdf+self.w[i]/accum
            self.cdf[i]=sum_cdf
        return (indexes_sorted)

    def create_uniform(indexes_sorted):
        #uniform: use prescribed distribution function based on cumulative distribution function previously built
        #This way the particles are drawn from a distribution induced by the weights
        draw_uniform=np.random.uniform(0,1)
        for i in range(0,self.n_p):
            for j in range(0,self.n_p):
                if draw_uniform <= self.cdf[j]:
                    self.x_particle_resampled[i]=self.x_particle_next[indexes_sorted[j]]
                    self.y_particle_resampled[i]=self.y_particle_next[indexes_sorted[j]]
                    break

    def delta_output(cx_identified, cy_identified):
        if self.initialize_flag==0: #This tracker is being shutdown and it will go through a new vehicle assignment process
            self.delta_x_out=0
            self.delta_y_out=0
        else:
            self.delta_x_out=cx_identified - self.centroid_x_previous
            self.delta_y_out=cy_identified - self.centroid_y_previous

    def resample_particles():
        # create_cdf
        indexes_sorted, self.cdf, self.w = create_cdf(self.distance_to_particle_identified, self.w, self.cdf)
        # create uniform
        create_uniform(indexes_sorted)


    def tracker(Lc, trackerID):
        mean = [self.x_particles_input + self.delta_x_input, self.y_particles_input+self.delta_y_input]  #mean is 1 x 2*n_p array
        cov = [[n_p, 0], [0, self.n_p]]
        self.x_particle_next, self.y_particle_next = np.multivariate_normal(mean, cov)

        cx_identified = 0
        cy_identified = 0

        # #Generation of 10 random particles
        # for k1 in range(0,10):
        #     mean = [x_particles_input[k1]+delta_x_input, y_particles_input[k1]+delta_y_input]  #delta_x_input and delta_y_input are based on the trajectory vector
        #     cov = [[10, 0], [0, 10]]
        #     self.x_particle_next[k1], self.y_particle_next[k1] = np.random.multivariate_normal(mean, cov)
        #     cv2.circle(self.img,(int(self.x_particle_next[k1]), int(self.y_particle_next[k1])), 8, (0,0,255), -1)
        #Identifying the bounding box through the particles: which bounding box corresponds to these particles
        min_average_distance=10000
        for k in range(0,self.n_v):
            x1=self.d[k][1]
            y1=self.d[k][0]
            x2=self.d[k][3]
            y2=self.d[k][2]

            #centroid of the bounding box
            cx = (x1+x2)/2
            cy = (y1+y2)/2

            #determine the closest bounding box to the cloud of particles
            acum_distance_particles=0
            for k1 in range(0,10):
                self.distance_to_particle[k1]=((cx-self.x_particle_next[k1])**2+(cy-self.y_particle_next[k1])**2)**0.5
                acum_distance_particles = acum_distance_particles + self.distance_to_particle[k1]

            average_distance_to_particle = acum_distance_particles/10
            if average_distance_to_particle < min_average_distance:
                min_average_distance= average_distance_to_particle
                identified=k
                cx_identified=cx
                cy_identified=cy
                for j1 in range(0,self.n_p):
                    self.distance_to_particle_identified[j1]=self.distance_to_particle[j1]

        if  self.count_holding_input==0:
            if abs(cx_identified-self.centroid_x_previous)>100: #the bounding box possibly dissapeared
                self.count_holding_input=self.count_holding_input+1
                cv2.rectangle(self.img, (int(self.centroid_x_previous-20),int(self.centroid_y_previous-20)), (int(self.centroid_x_previous+20),int(self.centroid_y_previous+20)), (0,255,0), 2, 1)
                cv2.rectangle(self.img, (int(cx_identified-20),int(cy_identified-20)), (int(cx_identified+20),int(cy_identified+20)), (0,255,0), 2, 1)
                cx_identified=self.centroid_x_previous
                cy_identified=self.centroid_y_previous
                self.distance_to_particle_identified=self.distance_to_particle_identified_previous_input
                self.delta_x_holding=self.delta_x_input


            else:
                for k4 in range(1,self.count_tracked_vehicles+1):
                    if k4 != trackerID:
                        if ((cx_identified-self.cx_tracked[k4])**2+(cy_identified-self.cy_tracked[k4])**2)**0.5<50 and self.count_holdingehicles[k4]==0: #due to bounding box dissapearance another bounding box corresponding to a different vehicle (another trackerID) may have been assigned to the current vehicle
                            count_holding_input=count_holding_input+1
                            cv2.rectangle(self.img, (int(self.cx_tracked[k4]-20),int(self.cy_tracked[k4]-20)), (int(self.cx_tracked[k4]+20),int(self.cy_tracked[k4]+20)), (255,0,0), 2, 1)
                            cv2.rectangle(self.img, (int(cx_identified-20),int(cy_identified-20)), (int(cx_identified+20),int(cy_identified+20)), (255,255,255), 2, 1)
                            #override current identification of particles (in other words override current bounding box assigment) and use data from previous successful bounding box assigment while we hold waiting for the bounding box to reappear
                            cx_identified=self.centroid_x_previous
                            cy_identified=self.centroid_y_previous
                            self.distance_to_particle_identified=self.distance_to_particle_identified_previous_input
                            self.delta_x_holding=self.delta_x_input

                            break
        else:   #we are in a process of holding for the bounding box to appear
            if self.count_holding_input>=4: #if bounding box doesn't appear after 4 frames then shutdown the current tracker assigned to the box
                self.count_holding_input=0
                self.initialize_flag=0 #this tracker will go through a brand new assignment process

            else: #we are waiting for the bounding box to appear
                if abs(cx_identified-self.centroid_x_previous)>100:
                    self.count_holding_input=self.count_holding_input+1
                    cx_identified=self.centroid_x_previous
                    cy_identified=self.centroid_y_previous
                    self.distance_to_particle_identified=self.distance_to_particle_identified_previous_input
                    #we still keep the same delta_x_holding from previous holdings
                else: #the bounding box corresponding to this tracker may have reappeared
                    merging_conflict=0
                    for k4 in range(1,self.count_tracked_vehicles+1):
                        if k4 != trackerID:
                            if ((cx_identified-self.cx_tracked[k4])**2+(cy_identified-self.cy_tracked[k4])**2)**0.5<50: #there is conflict with another tracker, bounding box assignment is not possible so we keep holding
                                self.count_holding_input=self.count_holding_input+1
                                cx_identified=self.centroid_x_previous
                                cy_identified=self.centroid_y_previous
                                self.distance_to_particle_identified=self.distance_to_particle_identified_previous_input
                                merging_conflict=1
                                break
                    if  merging_conflict==0:
                        self.count_holding_input=0 #the bounding box reappeared

        #Displaying trackers
        display_trackers()
        ######### Resampling of Particles ###################
        resample_particles()
        delta_output(cx_identified, cy_identified)
        return cx_identified,cy_identified

    def uninitialized_vehicle(Lc):
        vehicle_found_for_initialization=0  #succesful initialization of this tracker
        vehicle_index_for_initialization=0
        for k2 in range(0,Lc):
            x1=self.d[k2][1]
            y1=self.d[k2][0]
            x2=self.d[k2][3]
            y2=self.d[k2][2]
            cx = (x1+x2)/2
            cy = (y1+y2)/2
            conflict=0
            for k3 in range(1,self.count_tracked_vehicles+1):
                if ((cx-self.cx_tracked[k3])**2+(cy-self.cy_tracked[k3])**2)**0.5<50:
                    conflict=1
                    break
                elif (self.count_holdingehicles[k3]>0 or self.cy_tracked[k3]>1600) and (((cx-cx_tracked[k3])**2+(cy-self.cy_tracked[k3])**2)**0.5<100): #if the vehicle being probed is holding then we need to be more restrictive because the holding is being done with cx_previous and cy_previous which is farther from current bounding box that in principle corresponds to holding vehicle
                    conflict=1
                    break
            if conflict==0:   #The bounding box being picked does not belong/correspond to another tracker
                self.cx_tracked[veh]=cx
                self.cy_tracked[veh]=cy
                vehicle_found_for_initialization=1  #succesful initialization of this tracker
                vehicle_index_for_initialization=k2
                break
        if vehicle_found_for_initialization==1:
            cv2.circle(self.img,(int(cx), int(cy)), 8, (0,0,255), -1)
            mean = [cx, cy]
            cov = [[100, 0], [0, 100]]
            self.cx_previous_vehicles[veh]=cx
            self.cy_previous_vehicles[veh]=cy
            for k1 in range(0,10):  #10 particles assigned to this bounding box (vehicle)
                self.x_particle_vehicles[veh][k1], self.y_particle_vehicles[veh][k1] = np.random.multivariate_normal(mean, cov)
                cv2.circle(self.img,(int(self.x_particle_vehicles[veh][k1]), int(self.y_particle_vehicles[veh][k1])), 8, (255,0,0), -1)
            cv2.rectangle(self.img, (self.d[vehicle_index_for_initialization][1],self.d[vehicle_index_for_initialization][0]), (self.d[vehicle_index_for_initialization][3],self.d[vehicle_index_for_initialization][2]), (255,255,255), 2, 1)
            print("trackerID: ",veh)
            initialize_vehicles[veh]=1 #A Tracker has been assigned to this vehicle

    def initialized_vehicle(veh):
        cx_identified_out,cy_identified_out,initialize_vehicles[veh] = tracker(Lc,veh)
        #Information from current frame to be carried to next frame (useful when performing holding)
        self.cx_previous_vehicles[veh]=cx_identified_out
        self.cy_previous_vehicles[veh]=cy_identified_out
        self.distance_to_particle_identified_previous_vehicles[veh]=self.distance_to_particle_identified_out
        if initialize_vehicles[veh]==0:
            self.cx_tracked[veh]=0
            self.cy_tracked[veh]=0
        else:
            self.cx_tracked[veh]=cx_identified_out
            self.cy_tracked[veh]=cy_identified_out

    #######  Tracking   #############
    def tracking_main():
        a=box_info[index-1080]
        box_centers=a['centers']
        c=a['areas']
        Lc=len(c)
        d=a['coords']

        for k in range(0,Lc):
            cv2.rectangle(frame2, (self.d[k][1],self.d[k][0]), (self.d[k][3],self.d[k][2]), (0,0,0), 2, 1)
        index += 1
        for veh in range(1,self.count_tracked_vehicles+1):
            #########  Tracker Initialization  #############
            ##pick on vehicle
            if initialize_vehicles[veh]==0:
                uninitialized_vehicle(Lc)
            else:
                initialized_vehicle(veh)
            cv2.namedWindow('Tracking',cv2.WINDOW_NORMAL)
            cv2.imshow("Tracking", self.img)
            cv2.waitKey(1)
