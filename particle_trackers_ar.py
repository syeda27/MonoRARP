import cv2
import math
import numpy as np
import pickle

# JUAN CARLOS' version
# edited by Anjali

class ParticleTracker:
    def __init__(self, n_p, n_v):
        # n_p is number of particles.
        # n_v is the max number of trackers
        self.n_p = n_p
        self.n_v = n_v
        self.particles = np.zeros((n_v, n_p, 2)) # 2 because x, y
        self.max_holding = 3 # TODO args
        self.max_tracker_jump = 0.1 # TODO args
        self.tracked_boxes = np.zeros((n_v, 4)) # box coordinates. index is id
        self.box_indices = set() # the box labels
        self.distance_to_particle_identified = np.zeros((n_v, n_p))
        self.previous_distance_to_particle_identified = np.zeros((n_v, n_p))
        # need to track previous in case we revert (grep for merge_conflict)

        self.initialized_trackers=np.zeros(n_v)
        self.delta_x_trackers=np.zeros(n_v)
        self.delta_y_trackers=np.zeros(n_v)

        self.count_holding_vehicles=np.zeros(n_v)
        self.centroid_x_previous=np.zeros(n_v)
        self.centroid_y_previous=np.zeros(n_v)

        # to be defined in create
        self.img = None
        self.detections = None

        self.display = False # TODO args

    def _display_trackers(self, trackerID, identified):
        if self.initialized_trackers[trackerID] == 1 and \
                self.count_holding_vehicles[trackerID] == 0:
            colors = [(0,0,255), (0,255,0), (255,0,0),(150,100,0)]
            cv2.rectangle(
                self.img,
                (self.detections[identified][1],self.detections[identified][0]),
                (self.detections[identified][3],self.detections[identified][2]),
                math.ceil(colors[trackerID / 2]),
                2,
                1)
            cv2.putText(
                self.img,
                str(trackerID),
                (int(cx_identified),int(cy_identified)),
                cv2.FONT_HERSHEY_SIMPLEX,
                2,
                (0,255,0),
                2)

    def resample_particles(self, trackerID):
        #reindexing
        indexes_sorted=[b[0] for b in sorted(
                enumerate(self.distance_to_particle_identified[trackerID]),
                key=lambda i:i[1])]
        #weights
        w = [math.exp(-0.5*(
                self.distance_to_particle_identified[trackerID, indexes_sorted[i]]**0.5
            )) for i in range(self.n_p)]
        accum = np.sum(w)
        #cumulative distribution
        cdf = np.zeros(self.n_p)
        sum_cdf = 0
        for i in range(self.n_p):
            sum_cdf += w[i] / accum
            cdf[i]=sum_cdf
        #uniform: use prescribed distribution function based on cumulative distribution function previously built
        #This way the particles are drawn from a distribution induced by the weights
        temp_particles = self.particles[trackerID,:,:]
        for i in range(self.n_p):
            draw_uniform = np.random.uniform(0,1)
            for j in range(self.n_p):
                if draw_uniform <= cdf[j]:
                    self.particles[trackerID, i, 0] = temp_particles[indexes_sorted[j], 0]
                    self.particles[trackerID, i, 1] = temp_particles[indexes_sorted[j], 1]
                    break

    def gen_rand_particls(self, trackerID):
        for i in range(self.n_p):
            mean = [
                self.particles[trackerID,i,0] + self.delta_x_trackers[trackerID],
                self.particles[trackerID,i,1] + self.delta_y_trackers[trackerID]
            ]
            cov = [[self.n_p, 0], [0, self.n_p]]
            (self.particles[trackerID,:,0],
             self.particles[trackerID,:,1]) = np.random.multivariate_normal(mean, cov)

    def identify_particles_bboxes(self, trackerID):
        self.gen_rand_particls(trackerID)

        cx_identified, cy_identified, identified = 0, 0, 0
        distance_to_particle_identified = np.zeros(self.n_p)
        min_average_distance = 10000
        for box_index in range(len(self.detections)):
            distance_to_particle = np.zeros(self.n_p)
            x1 = self.detections[box_index][1]
            y1 = self.detections[box_index][0]
            x2 = self.detections[box_index][3]
            y2 = self.detections[box_index][2]
            #centroid of the bounding box
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2

            #determine the closest bounding box to the cloud of particles
            acum_distance_particles = 0
            for particle_index in range(self.n_p):  # TODO args
                distance_to_particle[particle_index] = (
                    (cx - self.particles[trackerID, particle_index, 0])**2 +
                    (cy - self.particles[trackerID, particle_index, 1])**2
                )**0.5
                acum_distance_particles += distance_to_particle[particle_index]
            average_distance_to_particle = acum_distance_particles / self.n_p
            if average_distance_to_particle < min_average_distance:
                min_average_distance = average_distance_to_particle
                identified = box_index
                cx_identified = cx
                cy_identified = cy
                for particle_index in range(self.n_p): # To store, if found.
                    self.distance_to_particle_identified[trackerID, particle_index] = \
                        distance_to_particle[particle_index]
        return cx_identified, cy_identified, identified

    def is_merge_conflict(self, trackerID, cx_identified, cy_identified, init=False):
        #if trackerID > 0: return True
        #return False
        for other_tracker_id in range(self.n_v):
            if other_tracker_id == trackerID or \
            self.initialized_trackers[trackerID] == 0:
                # check other trackers
                continue
            distance_to_box = (
                (cx_identified - self.centroid_x_previous[other_tracker_id])**2 +
                (cy_identified - self.centroid_y_previous[other_tracker_id])**2
            )**0.5
            print("d:", distance_to_box, "d:", self.max_tracker_jump / 2 * \
                    (self.count_holding_vehicles[other_tracker_id] + 1))
            print(init, trackerID, other_tracker_id)

            if init:
                raise ValueError
                if distance_to_box < self.max_tracker_jump / 2 * \
                        (self.count_holding_vehicles[other_tracker_id] + 1):
                    print("conflict")
                    raise ValueError
                    return True
            elif distance_to_box < self.max_tracker_jump / 2 and \
                    self.count_holding_vehicles[other_tracker_id] == 0:
                # If the box is close to some other tracker that is not holding, we hold.
                print("conflict no init")
                return True
        return False

    def reset_tracker(self, trackerID):
        self.count_holding_vehicles[trackerID] = 0
        self.initialized_trackers[trackerID] = 0
        self.centroid_x_previous[trackerID] = 0
        self.centroid_y_previous[trackerID] = 0
        self.delta_x_trackers[trackerID] = 0
        self.delta_y_trackers[trackerID] = 0

    def increment_holding(self, trackerID):
        """
        keep data from previous successful bounding box
        assigment while we hold waiting for the bounding box to reappear
        """
        self.count_holding_vehicles[trackerID] += 1
        #cx_identified = self.centroid_x_previous[trackerID]
        #cy_identified = self.centroid_y_previous[trackerID]
        self.distance_to_particle_identified[trackerID] = \
            self.previous_distance_to_particle_identified[trackerID]
        if self.count_holding_vehicles[trackerID] > self.max_holding:
            self.reset_tracker(trackerID)

    def update_tracker(self, trackerID, cx_identified, cy_identified, box_index):
        if self.initialized_trackers[trackerID] > 0:
            self.delta_x_trackers[trackerID] = (
                cx_identified - self.centroid_x_previous[trackerID]
            ) / (self.count_holding_vehicles[trackerID] + 1)
            self.delta_y_trackers[trackerID] = (
                cy_identified - self.centroid_y_previous[trackerID]
            ) / (self.count_holding_vehicles[trackerID] + 1) # The average movement
        self.count_holding_vehicles[trackerID] = 0 #the bounding box reappeared
        self.centroid_x_previous[trackerID] = cx_identified
        self.centroid_y_previous[trackerID] = cy_identified
        self.initialized_trackers[trackerID] = 1
        self.update_tracked_boxes(trackerID, box_index)

    def update_tracked_boxes(self, trackerID, box_index):
        self.tracked_boxes[trackerID] = self.detections[box_index]
        self.box_indices.add(box_index)
        # TODO move to new centroid and average dimensions?

    def update_initialized_tracker(self, trackerID):
        #Identifying the bounding box through the particles: which bounding box corresponds to these particles
        cx_identified, cy_identified, identified = self.identify_particles_bboxes(trackerID)
        x_translation = abs(cx_identified - self.centroid_x_previous[trackerID])
        if x_translation > self.max_tracker_jump or \
                self.is_merge_conflict(trackerID, cx_identified, cy_identified):
            #the bounding box possibly dissapeared
            self.increment_holding(trackerID)
        else:
            self.update_tracker(trackerID, cx_identified, cy_identified, identified)
        self.resample_particles(trackerID)

    def try_to_start_tracking(self, trackerID):
        """
        Must be run after updating all current trackers.
        """
        for box_index in range(len(self.detections)):
            if box_index in self.box_indices: continue
            x1 = self.detections[box_index][1]
            y1 = self.detections[box_index][0]
            x2 = self.detections[box_index][3]
            y2 = self.detections[box_index][2]
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            if self.verbose:
                print("Box {} with centroid: {}, {}".format(
                    box_index, cx, cy
                ))
            if not self.is_merge_conflict(trackerID, cx, cy, init=True):
                """
                if the vehicle being probed is holding then we need to be
                more restrictive because the holding is being done with
                cx_previous and cy_previous which is farther from current
                bounding box that in principle corresponds to holding vehicle
                """
                #The bounding box being picked does not belong/correspond to another tracker
                mean = [cx, cy]
                cov = [[100, 0], [0, 100]]
                (self.particles[trackerID,:,0],
                 self.particles[trackerID,:,1]) = np.random.multivariate_normal(mean, cov)
                self.update_tracker(trackerID, cx, cy, box_index)
                if self.verbose:
                    print("tracker {} created for box {}".format(
                        trackerID, box_index))
                return

    def reset_all_trackers(self):
        for trackerID in range(self.n_v):
            self.reset_tracker(trackerID)

    def get_boxes(self):
        boxes = []
        for trackerID in range(self.n_v):
            if self.initialized_trackers[trackerID] == 1:
                boxes.append(self.tracked_boxes[trackerID])
        # TODO return object IDs
        return boxes

    def update_all(self, image, boxes, verbose=False):
        self.img = image
        self.detections = boxes
        self.verbose = True
        self.box_indices = set()
        for trackerID in range(self.n_v):
            if self.initialized_trackers[trackerID] == 1:
                self.update_initialized_tracker(trackerID)
        for trackerID in range(self.n_v):
            if self.initialized_trackers[trackerID] == 0:
                self.try_to_start_tracking(trackerID)
        return self.get_boxes()
