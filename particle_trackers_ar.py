import cv2
import math
import numpy as np
import pickle
import copy

# JUAN CARLOS' version
# edited by Anjali

class ParticleTracker:
    def __init__(self,
            num_particles,
            num_trackers,
            max_holding=2,
            max_tracker_jump=0.1,
            cov=0.0001,
            min_allowable_likelihood=-0.25):
        # num_particles is number of particles.
        # num_trackers is the max number of trackers
        self.num_particles = num_particles
        self.num_trackers = num_trackers
        self.max_holding = max_holding
        self.max_tracker_jump = max_tracker_jump
        self.cov = cov
        self.cov_x_arr = np.diag([self.cov]*self.num_particles)
        self.cov_y_arr = np.diag([self.cov/2]*self.num_particles)
        # TODO have higher variance for bigger objects?
        # TODO have some way to learn the covariance over time.

        self.min_allowable_likelihood = min_allowable_likelihood
        # especially necessary for alternatives

        self.particles = np.zeros((num_trackers, num_particles, 2)) # 2 because x, y
        self.tracked_boxes = np.zeros((num_trackers, 4)) # box coordinates. index is id
        self.tracked_labels = ["" for i in range(num_trackers)]
        self.box_indices = set() # the box labels
        self.distance_to_particle_identified = np.zeros((num_trackers, num_particles))
        self.previous_distance_to_particle_identified = np.zeros((num_trackers, num_particles))
        # need to track previous in case we revert (grep for merge_conflict)

        self.initialized_trackers=np.zeros(num_trackers)
        self.delta_x_trackers=np.zeros(num_trackers)
        self.delta_y_trackers=np.zeros(num_trackers)

        self.count_holding_vehicles=np.zeros(num_trackers)
        self.centroid_x_previous=np.zeros(num_trackers)
        self.centroid_y_previous=np.zeros(num_trackers)

        # to be defined in create
        self.img = None
        self.detections = None
        self.box_indices = set()

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

    def _reset_tracker(self, trackerID):
        self.count_holding_vehicles[trackerID] = 0
        self.initialized_trackers[trackerID] = 0
        self.centroid_x_previous[trackerID] = 0
        self.centroid_y_previous[trackerID] = 0
        self.delta_x_trackers[trackerID] = 0
        self.delta_y_trackers[trackerID] = 0

    def _resample_particles(self, trackerID):
        """
        For this tracker, shuffle the particles based on the cdf and the
        distance to the identified particle.
        """
        #reindexing
        indexes_sorted=[b[0] for b in sorted(
                enumerate(self.distance_to_particle_identified[trackerID]),
                key=lambda i:i[1])]
        #weights
        w = [math.exp(-0.5*(
                self.distance_to_particle_identified[trackerID, indexes_sorted[i]]**0.5
            )) for i in range(self.num_particles)]
        accum = np.sum(w)
        #cumulative distribution
        cdf = np.zeros(self.num_particles)
        sum_cdf = 0
        for i in range(self.num_particles):
            sum_cdf += w[i] / accum
            cdf[i]=sum_cdf
        #uniform: use prescribed distribution function based on cumulative distribution function previously built
        #This way the particles are drawn from a distribution induced by the weights
        temp_particles = self.particles[trackerID,:,:]
        for i in range(self.num_particles):
            draw_uniform = np.random.uniform(0,1)
            for j in range(self.num_particles):
                if draw_uniform <= cdf[j]:
                    self.particles[trackerID, i, 0] = temp_particles[indexes_sorted[j], 0]
                    self.particles[trackerID, i, 1] = temp_particles[indexes_sorted[j], 1]
                    break
        if self.verbose:
            print("Tracker {}\nOld Particles{}\nNew Particles{}".format(
                trackerID,
                temp_particles,
                self.particles[trackerID, :, :]
            ))

    def gen_rand_particles(self, trackerID):
        """
        create num_particles random particles for this tracker using the current particles
        plus delta as the mean and a multivariate normal distr.
        """
        mean_x = self.particles[trackerID,:,0] + self.delta_x_trackers[trackerID]
        mean_y = self.particles[trackerID,:,1] + self.delta_y_trackers[trackerID]
        self.particles[trackerID,:,0] = np.clip(
            np.random.multivariate_normal(
                mean_x,
                self.cov_x_arr * (self.tracked_boxes[trackerID][3] - self.tracked_boxes[trackerID][1])
            ),
            0.0,
            1.0)
        # TODO scale cov_x_arr by the box width?
        self.particles[trackerID,:,1] = np.clip(
            np.random.multivariate_normal(mean_y, self.cov_y_arr),
            0.0,
            1.0)


    def identify_particles_bboxes(self, trackerID):
        """
        For a given tracker, figure out which detection is most likely being
        tracked by us. There must be self.particles for this trackerID
        """
        self.gen_rand_particles(trackerID)
        cx_identified, cy_identified, identified = 0, 0, 0
        distance_to_particle_identified = np.zeros(self.num_particles)
        max_likelihood = -10000
        for box_index in range(len(self.detections)):
            likelihood, cx, cy, d_particles = self.likelihood_of_detection(
                trackerID, box_index)
            if likelihood > max_likelihood:
                cx_identified = cx
                cy_identified = cy
                identified = box_index
                max_likelihood = likelihood
                for particle_index in range(self.num_particles): # To store, if found.
                    self.distance_to_particle_identified[trackerID, particle_index] = \
                        d_particles[particle_index]
        return cx_identified, cy_identified, identified

    def likelihood_of_detection(self, trackerID, box_index):
        """
        likelihood here is not in the mathematical sense, but could be something
        as simple as the inverse distance (lower distance -> higher likelihood).
        This can be extended later to more mathematical representations.
        """
        x1 = self.detections[box_index][1]
        y1 = self.detections[box_index][0]
        x2 = self.detections[box_index][3]
        y2 = self.detections[box_index][2]
        #centroid of the bounding box
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2

        #determine the closest bounding box to the cloud of particles
        if self.verbose:
            print("Tracker ID {} and Box {}".format(trackerID, box_index))
            print(self.particles[trackerID, :, :])
            print(cx, cy)
        distances_to_particles = [np.linalg.norm(
            [cx, cy] - self.particles[trackerID, p, :]
        ) for p in range(self.num_particles)]
        likelihood = -np.mean(distances_to_particles)
        # TODO likelihood /= (x2 - x1) # divide by box width. more movement ok if big box.
        # TODO incorporate change in box size?
        return likelihood, cx, cy, distances_to_particles


    def is_merge_conflict(self, trackerID, cx_identified, cy_identified):
        # TODO must something else be done if we want to initialize?
        for other_tracker_id in range(self.num_trackers):
            if other_tracker_id == trackerID or \
            self.initialized_trackers[trackerID] == 0:
                # check other trackers
                continue
            distance_to_box = np.linalg.norm([
                cx_identified - self.centroid_x_previous[other_tracker_id],
                cy_identified - self.centroid_y_previous[other_tracker_id]
            ])
            if self.verbose:
                print("d:", distance_to_box, "d:", self.max_tracker_jump / 2 * \
                        (self.count_holding_vehicles[other_tracker_id] + 1))
                print(init, trackerID, other_tracker_id)

            if distance_to_box < self.max_tracker_jump / 2 and \
                    self.count_holding_vehicles[other_tracker_id] == 0:
                # If the box is close to some other tracker that is not holding, we hold.
                if self.verbose:
                    print("conflict no init")
                return True
        return False

    def increment_holding(self, trackerID):
        """
        keep data from previous successful bounding box
        assigment while we hold waiting for the bounding box to reappear
        """
        if self.count_holding_vehicles[trackerID] >= self.max_holding:
            self._reset_tracker(trackerID)
            return
        if self.verbose:
            print("Holding # {} for tracker: {}".format(
                self.count_holding_vehicles[trackerID],
                trackerID)
            )
        self.count_holding_vehicles[trackerID] += 1
        self.distance_to_particle_identified[trackerID] = \
            self.previous_distance_to_particle_identified[trackerID]
        self.tracked_boxes[trackerID] += [
            self.delta_x_trackers[trackerID],
            self.delta_y_trackers[trackerID],
            self.delta_x_trackers[trackerID],
            self.delta_y_trackers[trackerID]]
        # move the box by our expected movement, for visualization purposes.

    def update_tracker(self, trackerID, cx_identified, cy_identified, box_index):
        if self.initialized_trackers[trackerID] == 1:
            self.delta_x_trackers[trackerID] = (
                cx_identified - self.centroid_x_previous[trackerID]
            ) / (self.count_holding_vehicles[trackerID] + 1)
            self.delta_y_trackers[trackerID] = (
                cy_identified - self.centroid_y_previous[trackerID]
            ) / (self.count_holding_vehicles[trackerID] + 1) # The average movement
        if self.verbose:
            print("Delta for tracker ", trackerID)
            print(self.delta_x_trackers[trackerID])
            print(self.delta_y_trackers[trackerID])
        self.count_holding_vehicles[trackerID] = 0 #the bounding box reappeared
        self.centroid_x_previous[trackerID] = cx_identified
        self.centroid_y_previous[trackerID] = cy_identified
        self.initialized_trackers[trackerID] = 1
        self.update_tracked_boxes(trackerID, box_index)
        self._resample_particles(trackerID)

    def update_tracked_boxes(self, trackerID, box_index):
        self.tracked_boxes[trackerID] = self.detections[box_index]
        self.tracked_labels[trackerID] = self.labels[box_index]
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
            if not self.is_merge_conflict(trackerID, cx, cy):
                """
                if the vehicle being probed is holding then we need to be
                more restrictive because the holding is being done with
                cx_previous and cy_previous which is farther from current
                bounding box that in principle corresponds to holding vehicle
                """
                #The bounding box being picked does not belong/correspond to another tracker
                mean = [cx, cy]
                cov = [[self.cov, 0], [0, self.cov]]
                (self.particles[trackerID,:,0],
                 self.particles[trackerID,:,1]) = np.random.multivariate_normal(mean, cov)
                self.update_tracker(trackerID, cx, cy, box_index)
                if self.verbose:
                    print("tracker {} created for box {}".format(
                        trackerID, box_index))
                return

    def reset_all_trackers(self):
        for trackerID in range(self.num_trackers):
            self._reset_tracker(trackerID)

    def get_boxes(self, with_holding=True):
        boxes_with_labels = dict()
        for trackerID in range(self.num_trackers):
            if self.initialized_trackers[trackerID] == 1 and \
                    (with_holding == True or self.count_holding_vehicles[trackerID] == 0):
                boxes_with_labels[trackerID] = (
                    self.tracked_boxes[trackerID],
                    self.tracked_labels[trackerID])
        if self.verbose:
            print("Returning {} boxes".format(len(boxes_with_labels.keys())))
        return boxes_with_labels

    def update_all(self, image, boxes, labels=None, verbose=False):
        if type(boxes) == type(None):
            return self.get_boxes()
        self.img = image
        self.detections = boxes
        self.verbose = verbose
        self.box_indices = set()
        self.labels = labels
        for trackerID in range(self.num_trackers):
            if self.initialized_trackers[trackerID] == 1:
                self.update_initialized_tracker(trackerID)
        for trackerID in range(self.num_trackers):
            if self.initialized_trackers[trackerID] == 0:
                self.try_to_start_tracking(trackerID)
        return self.get_boxes()
