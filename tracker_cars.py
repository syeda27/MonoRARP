import cv2
import math
import numpy as np
import pickle

# JUAN CARLOS' version
# edited by Anjali

def init_tracker(n_p):
    x_particle_out=np.zeros(n_p)
    y_particle_out=np.zeros(n_p)
    x_particle_next=np.zeros(n_p)
    y_particle_next=np.zeros(n_p)
    distance_to_particle=np.zeros(n_p)
    distance_to_particle_identified=np.zeros(n_p)

    w=np.zeros(n_p)     #weights to be used for the resampling part of Importance Sampling
    cdf=np.zeros(n_p)   #vector to be used for the cumulative distribution function

    x_particle_resampled=np.zeros(10)
    y_particle_resampled=np.zeros(10)

def display_trackers():
    if (trackerID==1 or trackerID==2) and initialize_flag==1 and count_holding_input==0:
        cv2.rectangle(frame2, (d[identified][1],d[identified][0]), (d[identified][3],d[identified][2]), (0,0,255), 2, 1)
        cv2.putText(frame2,str(trackerID),(int(cx_identified),int(cy_identified)), cv2.FONT_HERSHEY_SIMPLEX, 2,(0,255,0),2)
    if (trackerID==3 or trackerID==4) and initialize_flag==1 and count_holding_input==0:
        cv2.rectangle(frame2, (d[identified][1],d[identified][0]), (d[identified][3],d[identified][2]), (0,255,0), 2, 1)
        cv2.putText(frame2,str(trackerID),(int(cx_identified),int(cy_identified)), cv2.FONT_HERSHEY_SIMPLEX, 2,(0,255,0),2)
    if (trackerID==5 or trackerID==6) and initialize_flag==1 and count_holding_input==0:
        cv2.rectangle(frame2, (d[identified][1],d[identified][0]), (d[identified][3],d[identified][2]), (255,0,0), 2, 1)
        cv2.putText(frame2,str(trackerID),(int(cx_identified),int(cy_identified)), cv2.FONT_HERSHEY_SIMPLEX, 2,(0,255,0),2)
    if (trackerID==7 or trackerID==8) and initialize_flag==1 and count_holding_input==0:
        cv2.rectangle(frame2, (d[identified][1],d[identified][0]), (d[identified][3],d[identified][2]), (150,100,0), 2, 1)
        cv2.putText(frame2,str(trackerID),(int(cx_identified),int(cy_identified)), cv2.FONT_HERSHEY_SIMPLEX, 2,(0,255,0),2)

def create_cdf(distance_to_particle_identified, w, cdf):
    #reindexing
    indexes_sorted=[b[0] for b in sorted(enumerate(distance_to_particle_identified),key=lambda i:i[1])]
    #weights
    accum=0
    for i in range(0,10):
        w[i]=math.exp(-0.5*((distance_to_particle_identified[indexes_sorted[i]])**0.5))
        accum=accum+w[i]
    #cumulative distribution
    sum_cdf=0
    for i in range(0,10):
        sum_cdf=sum_cdf+w[i]/accum
        cdf[i]=sum_cdf
    return (indexes_sorted, cdf, w)

def create_uniform(n_p, indexes_sorted, cdf, x_particle_resampled, y_particle_resampled):
    #uniform: use prescribed distribution function based on cumulative distribution function previously built
    #This way the particles are drawn from a distribution induced by the weights
    draw_uniform=np.random.uniform(0,1)
    for i in range(0,n_p):
        for j in range(0,n_p):
            if draw_uniform <= cdf[j]:
                x_particle_resampled[i]=x_particle_next[indexes_sorted[j]]
                y_particle_resampled[i]=y_particle_next[indexes_sorted[j]]
                break

def delta_output(initialize_flag, cx_identified, cy_identified, centroid_x_previous, centroid_y_previous):
    if initialize_flag==0: #This tracker is being shutdown and it will go through a new vehicle assignment process
        delta_x_out=0
        delta_y_out=0
    else:
        delta_x_out=cx_identified - centroid_x_previous
        delta_y_out=cy_identified - centroid_y_previous
        initialization_delta=1
    return delta_x_out, delta_y_out

def resample_particles(distance_to_particle_identified, w, cdf, x_particle_resampled, y_particle_resampled):
    # create_cdf
    indexes_sorted, cdf, w = create_cdf(distance_to_particle_identified, w, cdf)
    # create uniform
    create_uniform(n_p, indexes_sorted, cdf, x_particle_resampled, y_particle_resampled)

    return x_particle_resampled, y_particle_resampled

def tracker(x_particles_input,y_particles_input,delta_x_input,delta_y_input,Number_of_objects,centroid_x_previous,centroid_y_previous,initialize_flag,count_holding_input,distance_to_particle_identified_previous_input,trackerID):
    n_p = 10 #n_p = number of particles
    (x_particle_out, y_particle_out, x_particle_next, y_particle_next, distance_to_particle,
    distance_to_particle_identified, w, cdf, x_particle_resampled, y_particle_resampled) = init_tracker(n_p)

    mean = [x_particles_input + delta_x_input, y_particles_input+delta_y_input]  #mean is 1 x 2*n_p array
    cov = [[n_p, 0], [0, n_p]]
    x_particle_next, y_particle_next = np.multivariate_normal(mean, cov)
    #
    # #Generation of 10 random particles
    # for k1 in range(0,10):
    #     mean = [x_particles_input[k1]+delta_x_input, y_particles_input[k1]+delta_y_input]  #delta_x_input and delta_y_input are based on the trajectory vector
    #     cov = [[10, 0], [0, 10]]
    #     x_particle_next[k1], y_particle_next[k1] = np.random.multivariate_normal(mean, cov)
    #     cv2.circle(frame2,(int(x_particle_next[k1]), int(y_particle_next[k1])), 8, (0,0,255), -1)
    #Identifying the bounding box through the particles: which bounding box corresponds to these particles
    min_average_distance=10000
    for k in range(0,Number_of_objects):
        x1=d[k][1]
        y1=d[k][0]
        x2=d[k][3]
        y2=d[k][2]

        #centroid of the bounding box
        cx = (x1+x2)/2
        cy = (y1+y2)/2

        #determine the closest bounding box to the cloud of particles
        acum_distance_particles=0
        for k1 in range(0,10):
            distance_to_particle[k1]=((cx-x_particle_next[k1])**2+(cy-y_particle_next[k1])**2)**0.5
            acum_distance_particles=acum_distance_particles+distance_to_particle[k1]

        average_distance_to_particle=acum_distance_particles/10
        if average_distance_to_particle<min_average_distance:
            min_average_distance=average_distance_to_particle
            identified=k
            cx_identified=cx
            cy_identified=cy
            for j1 in range(0,10):
                distance_to_particle_identified[j1]=distance_to_particle[j1]

    if  count_holding_input==0:
        if abs(cx_identified-centroid_x_previous)>100: #the bounding box possibly dissapeared
            count_holding_input=count_holding_input+1
            cv2.rectangle(frame2, (int(centroid_x_previous-20),int(centroid_y_previous-20)), (int(centroid_x_previous+20),int(centroid_y_previous+20)), (0,255,0), 2, 1)
            cv2.rectangle(frame2, (int(cx_identified-20),int(cy_identified-20)), (int(cx_identified+20),int(cy_identified+20)), (0,255,0), 2, 1)
            cx_identified=centroid_x_previous
            cy_identified=centroid_y_previous
            distance_to_particle_identified=distance_to_particle_identified_previous_input
            delta_x_holding=delta_x_input


        else:
            for k4 in range(1,count_tracked_vehicles+1):
                if k4 != trackerID:
                    if ((cx_identified-cx_tracked[k4])**2+(cy_identified-cy_tracked[k4])**2)**0.5<50 and count_holding_vehicles[k4]==0: #due to bounding box dissapearance another bounding box corresponding to a different vehicle (another trackerID) may have been assigned to the current vehicle
                        count_holding_input=count_holding_input+1
                        cv2.rectangle(frame2, (int(cx_tracked[k4]-20),int(cy_tracked[k4]-20)), (int(cx_tracked[k4]+20),int(cy_tracked[k4]+20)), (255,0,0), 2, 1)
                        cv2.rectangle(frame2, (int(cx_identified-20),int(cy_identified-20)), (int(cx_identified+20),int(cy_identified+20)), (255,255,255), 2, 1)
                        #override current identification of particles (in other words override current bounding box assigment) and use data from previous successful bounding box assigment while we hold waiting for the bounding box to reappear
                        cx_identified=centroid_x_previous
                        cy_identified=centroid_y_previous
                        distance_to_particle_identified=distance_to_particle_identified_previous_input
                        delta_x_holding=delta_x_input

                        break
    else:   #we are in a process of holding for the bounding box to appear
        if count_holding_input>=4: #if bounding box doesn't appear after 4 frames then shutdown the current tracker assigned to the box
            count_holding_input=0
            initialize_flag=0 #this tracker will go through a brand new assignment process

        else: #we are waiting for the bounding box to appear
            if abs(cx_identified-centroid_x_previous)>100:
                count_holding_input=count_holding_input+1
                cx_identified=centroid_x_previous
                cy_identified=centroid_y_previous
                distance_to_particle_identified=distance_to_particle_identified_previous_input
                #we still keep the same delta_x_holding from previous holdings
            else: #the bounding box corresponding to this tracker may have reappeared
                merging_conflict=0
                for k4 in range(1,count_tracked_vehicles+1):
                    if k4 != trackerID:
                        if ((cx_identified-cx_tracked[k4])**2+(cy_identified-cy_tracked[k4])**2)**0.5<50: #there is conflict with another tracker, bounding box assignment is not possible so we keep holding
                            count_holding_input=count_holding_input+1
                            cx_identified=centroid_x_previous
                            cy_identified=centroid_y_previous
                            distance_to_particle_identified=distance_to_particle_identified_previous_input
                            merging_conflict=1
                            break
                if  merging_conflict==0:
                    count_holding_input=0 #the bounding box reappeared

    #Displaying trackers
    display_trackers()
    ######### Resampling of Particles ###################
    x_particle_out, y_particle_out = resample_particles(distance_to_particle_identified, w, cdf, x_particle_resampled, y_particle_resampled)
    delta_x_out,delta_y_out = delta_output(initialize_flag, cx_identified, cy_identified, centroid_x_previous, centroid_y_previous)

    return x_particle_out,y_particle_out,delta_x_out,delta_y_out,cx_identified,cy_identified,count_holding_input,distance_to_particle_identified,initialize_flag


#Intitialization of Variables
def init(n_v):
    # n_v = number of vehicles
    initialize_vehicles=np.zeros(n_v)
    x_particle=np.zeros(n_v)
    y_particle=np.zeros(n_v)
    distance_to_particle_identified_previous=np.zeros(n_v)
    distance_to_particle_identified_previous_vehicles=np.zeros((n_v,n_v))

    delta_x=0
    delta_y=0
    delta_x_vehicles=np.zeros(n_v)
    delta_y_vehicles=np.zeros(n_v)

    count_holding_vehicles=np.zeros(n_v)
    count_holding=0
    initialization_delta=0

    holding_tracked=np.zeros(n_v)
    count_tracked_vehicles=8 #We set the number of trackers that will work simultaneously
    cx_tracked=np.zeros(20)
    cy_tracked=np.zeros(20)

    cx_previous_vehicles=np.zeros(n_v)
    cy_previous_vehicles=np.zeros(n_v)

    img = cv2.imread('./hwy101_frames/1280.jpg',cv2.IMREAD_GRAYSCALE) #image for processing algorithm
    img3 = cv2.imread('./hwy101_frames/1280.jpg')                     #image for visualization of results (in color)
    with open('./Hwy101_box_info.pkl', 'rb') as f:
        box_info = pickle.load(f)
    index=1280

    return (index, box_info, cx_previous_vehicles, cy_previous_vehicles, cx_tracked, cy_tracked, count_tracked_vehicles, )

def uninitialized_vehicle(d, Lc, count_tracked_vehicles, cx_tracked, cy_tracked, count_holding_vehicles):
    vehicle_found_for_initialization=0  #succesful initialization of this tracker
    vehicle_index_for_initialization=0
    for k2 in range(0,Lc):
        x1=d[k2][1]
        y1=d[k2][0]
        x2=d[k2][3]
        y2=d[k2][2]
        cx = (x1+x2)/2
        cy = (y1+y2)/2
        conflict=0
        for k3 in range(1,count_tracked_vehicles+1):
            if ((cx-cx_tracked[k3])**2+(cy-cy_tracked[k3])**2)**0.5<50:
                conflict=1
                break
            else:
                if (count_holding_vehicles[k3]>0 or cy_tracked[k3]>1600) and (((cx-cx_tracked[k3])**2+(cy-cy_tracked[k3])**2)**0.5<100): #if the vehicle being probed is holding then we need to be more restrictive because the holding is being done with cx_previous and cy_previous which is farther from current bounding box that in principle corresponds to holding vehicle
                    conflict=1
                    break
        if conflict==0:   #The bounding box being picked does not belong/correspond to another tracker
            cx_tracked[veh]=cx
            cy_tracked[veh]=cy
            vehicle_found_for_initialization=1  #succesful initialization of this tracker
            vehicle_index_for_initialization=k2
            break
    if vehicle_found_for_initialization==1:
        cv2.circle(frame2,(int(cx), int(cy)), 8, (0,0,255), -1)
        mean = [cx, cy]
        cov = [[100, 0], [0, 100]]
        cx_previous_vehicles[veh]=cx
        cy_previous_vehicles[veh]=cy
        for k1 in range(0,10):  #10 particles assigned to this bounding box (vehicle)
            x_particle_vehicles[veh][k1], y_particle_vehicles[veh][k1] = np.random.multivariate_normal(mean, cov)
            cv2.circle(frame2,(int(x_particle_vehicles[veh][k1]), int(y_particle_vehicles[veh][k1])), 8, (255,0,0), -1)
        cv2.rectangle(frame2, (d[vehicle_index_for_initialization][1],d[vehicle_index_for_initialization][0]), (d[vehicle_index_for_initialization][3],d[vehicle_index_for_initialization][2]), (255,255,255), 2, 1)
        print("trackerID: ",veh)
        initialize_vehicles[veh]=1 #A Tracker has been assigned to this vehicle

def initialized_vehicle(veh):
    x_particle_vehicles[veh],y_particle_vehicles[veh],delta_x_vehicles[veh],delta_y_vehicles[veh],cx_identified_out,cy_identified_out,count_holding_vehicles[veh],distance_to_particle_identified_out,initialize_vehicles[veh] = tracker(x_particle_vehicles[veh],y_particle_vehicles[veh],delta_x_vehicles[veh],delta_y_vehicles[veh],Lc,cx_previous_vehicles[veh],cy_previous_vehicles[veh],initialize_vehicles[veh],count_holding_vehicles[veh],distance_to_particle_identified_previous_vehicles[veh],veh)
    #Information from current frame to be carried to next frame (useful when performing holding)
    cx_previous_vehicles[veh]=cx_identified_out
    cy_previous_vehicles[veh]=cy_identified_out
    distance_to_particle_identified_previous_vehicles[veh]=distance_to_particle_identified_out
    if initialize_vehicles[veh]==0:
        cx_tracked[veh]=0
        cy_tracked[veh]=0
    else:
        cx_tracked[veh]=cx_identified_out
        cy_tracked[veh]=cy_identified_out

#######  Tracking   #############
def tracking_main(n_v, n_p):
    while True:
        file = './hwy101_frames/'+str(index)+'.jpg'
        frame = cv2.imread(file,cv2.IMREAD_GRAYSCALE)
        frame2 = cv2.imread(file)

        a=box_info[index-1080]
        box_centers=a['centers']
        c=a['areas']
        Lc=len(c)
        d=a['coords']

        for k in range(0,Lc):
            cv2.rectangle(frame2, (d[k][1],d[k][0]), (d[k][3],d[k][2]), (0,0,0), 2, 1)
        index += 1
        for veh in range(1,count_tracked_vehicles+1):
            #########  Tracker Initialization  #############
            ##pick on vehicle
            if initialize_vehicles[veh]==0:
                uninitialized_vehicle((d, Lc, count_tracked_vehicles, cx_tracked, cy_tracked, count_holding_vehicles)
            else:
                initialized_vehicle(veh)
            cv2.namedWindow('Tracking',cv2.WINDOW_NORMAL)
            cv2.imshow("Tracking", frame2)
            cv2.waitKey(1)



initialize_vehicles=np.zeros(n_v)
x_particle=np.zeros(n_v)
y_particle=np.zeros(n_v)
distance_to_particle_identified_previous=np.zeros(n_v)
distance_to_particle_identified_previous_vehicles=np.zeros((n_v,n_v))

delta_x=0
delta_y=0
delta_x_vehicles=np.zeros(n_v)
delta_y_vehicles=np.zeros(n_v)

count_holding_vehicles=np.zeros(n_v)
count_holding=0
initialization_delta=0

holding_tracked=np.zeros(n_v)
count_tracked_vehicles=8 #We set the number of trackers that will work simultaneously
cx_tracked=np.zeros(20)
cy_tracked=np.zeros(20)

cx_previous_vehicles=np.zeros(n_v)
cy_previous_vehicles=np.zeros(n_v)

img = cv2.imread('./hwy101_frames/1280.jpg',cv2.IMREAD_GRAYSCALE) #image for processing algorithm
img3 = cv2.imread('./hwy101_frames/1280.jpg')                     #image for visualization of results (in color)
with open('./Hwy101_box_info.pkl', 'rb') as f:
    box_info = pickle.load(f)
index=1280
