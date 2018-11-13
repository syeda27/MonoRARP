
import numpy as np


class particle_tracker:
    def __init__(self, num_particles=10, id=1):
        self.tracker_id = id
        self.w = np.zeros(num_particles)
        self.cdf = np.zeros(num_particles)
        self.particles_x = np.zeros(num_particles)
        self.particles_y = np.zeros(num_particles)
        self.cov = [[num_particles, 0], [0, num_particles]]

    def gen_particles(self, delta_x_in, delta_y_in):
        for i in range(len(self.particles_x)):
            mean = [self.particles_x[i] + delta_x_input,
                    self.particles_y[i] + delta_y_input]
            #delta_x_input and delta_y_input are based on the trajectory vector
            self.particles_x[i], self.particles_y[i] = \
                np.random.multivariate_normal(mean, cov)

    def create(self, delta_x, delta_y):
        self.gen_particles(delta_x, delta_y)

def tracker(
    x_particles_input,
    y_particles_input,
    delta_x_input,
    delta_y_input,
    Number_of_objects,
    centroid_x_previous,
    centroid_y_previous,
    initialize_flag,
    count_holding_input,
    distance_to_particle_identified_previous_input,
    trackerID):

    x_particle_out=np.zeros(10)
    y_particle_out=np.zeros(10)
    x_particle_next=np.zeros(10)
    y_particle_next=np.zeros(10)
    distance_to_particle=np.zeros(10)
    distance_to_particle_identified=np.zeros(10)


    w=np.zeros(10)     #weights to be used for the resampling part of Importance Sampling
    cdf=np.zeros(10)   #vector to be used for the cumulative distribution function

    x_particle_resampled=np.zeros(10)
    y_particle_resampled=np.zeros(10)


    #Generation of 10 random particles
    for k1 in range(0,10):
        mean = [x_particles_input[k1]+delta_x_input, y_particles_input[k1]+delta_y_input]  #delta_x_input and delta_y_input are based on the trajectory vector
        cov = [[10, 0], [0, 10]]
        x_particle_next[k1], y_particle_next[k1] = np.random.multivariate_normal(mean, cov)
        cv2.circle(frame2,(int(x_particle_next[k1]), int(y_particle_next[k1])), 8, (0,0,255), -1)

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


    ######### Resampling of Particles ###################

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

    #uniform: use prescribed distribution function based on cumulative distribution function previously built
    #This way the particles are drawn from a distribution induced by the weights
    for i in range(0,10):
        draw_uniform=np.random.uniform(0,1)

        for j in range(0,10):

            if draw_uniform <= cdf[j]:
                x_particle_resampled[i]=x_particle_next[indexes_sorted[j]]
                y_particle_resampled[i]=y_particle_next[indexes_sorted[j]]

                break


    for k1 in range(0,10):

        x_particle_out[k1] = x_particle_resampled[k1]
        y_particle_out[k1] = y_particle_resampled[k1]

    if initialize_flag==0: #This tracker is being shutdown and it will go through a new vehicle assignment process
        delta_x_out=0
        delta_y_out=0
    else:
        delta_x_out=cx_identified-centroid_x_previous
        delta_y_out=cy_identified-centroid_y_previous
        initialization_delta=1


    return x_particle_out,y_particle_out,delta_x_out,delta_y_out,cx_identified,cy_identified,count_holding_input,distance_to_particle_identified,initialize_flag
