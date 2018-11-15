
from collections import defaultdict
import numpy as np
import particle_trackers_ar

class ParticleTrackerDP(particle_trackers_ar.ParticleTracker):

    def rank_detections_t(self, trackerID):
        # for this tracker, compute p(each detection and rank)
        self.ranked_detections_per_tracker[trackerID] = []
        for box_index in range(len(self.detections)):
            likelihood, cx, cy, d_particles = self.likelihood_of_detection(
                trackerID, box_index)
            self.ranked_detections_per_tracker[trackerID].append(
                (likelihood, box_index, d_particles))
        self.ranked_detections_per_tracker[trackerID].sort()
        # highest likelihood will be at the end. Makes removing easier. (.pop())

    def get_highest_likelihood_pair(self):
        """
        Guaranteed to return a pair that box that is not already tracked, or a
        match for a tracker that has already been assigned.
        """
        highest_likelihood = -1000000
        tracker_matched = -1
        box_index = -1
        best_d_particles = np.zeros(self.num_particles)
        for trackerID in self.unassigned_initialized_trackers:
            if len(self.ranked_detections_per_tracker[trackerID]) == 0:
                self.increment_holding(trackerID)
                self.unassigned_initialized_trackers.remove(trackerID)
                continue
            (likelihood, box_i, d_particles) = self.ranked_detections_per_tracker[trackerID][-1]
            if box_i in self.box_indices:
                try:
                    self.ranked_detections_per_tracker[trackerID].pop()
                except:
                    print(self.ranked_detections_per_tracker[trackerID])
                    print(likelihood, box_i)
                    raise ValueError
                continue
            if likelihood > highest_likelihood:
                highest_likelihood = likelihood
                tracker_matched = trackerID
                box_index = box_i
                best_d_particles = d_particles
        if tracker_matched > 0:
            self.ranked_detections_per_tracker[tracker_matched].pop()
        return highest_likelihood, tracker_matched, box_index, best_d_particles

    def increment_all_holding(self):
        for trackerID in self.unassigned_initialized_trackers:
            if self.verbose:
                print("Holding for ", trackerID)
            self.increment_holding(trackerID)

    def update_tracker_this_det(self, trackerID, box_index, d_particles):
        left = self.detections[box_index][1]
        top = self.detections[box_index][0]
        right = self.detections[box_index][3]
        bot = self.detections[box_index][2]
        cx = (left + right) / 2
        cy = (top + bot) / 2
        for particle_index in range(self.num_particles): # To store, if found.
            self.distance_to_particle_identified[trackerID, particle_index] = \
                d_particles[particle_index]
        self.update_tracker(trackerID, cx, cy, box_index)

    def update_all_initialized_trackers(self):
        self.unassigned_initialized_trackers = set()
        self.ranked_detections_per_tracker = defaultdict(list)
        for trackerID in range(self.num_trackers):
            if self.initialized_trackers[trackerID] == 1:
                self.unassigned_initialized_trackers.add(trackerID)
                self.gen_rand_particles(trackerID)
                self.rank_detections_t(trackerID)
        if self.verbose:
            for tracker_id, pairs in self.ranked_detections_per_tracker.items():
                for (l, box, d) in pairs:
                    print("tracker {} has likelihood {} for box {}".format(
                        tracker_id, l, box
                    ))
        while len(self.unassigned_initialized_trackers) > 0:
            (likelihood, trackerid, box_i, d_particles) = \
                self.get_highest_likelihood_pair()
            if trackerid < 0: return
            if likelihood < self.min_allowable_likelihood: # todo
                self.increment_all_holding()
                return
            self.update_tracker_this_det(trackerid, box_i, d_particles)
            self.unassigned_initialized_trackers.remove(trackerid)
            if self.verbose:
                print("tracker {} updated for det {}".format(
                    trackerid, box_i
                ))

    def try_to_start_tracking(self, trackerID):
        for box_index in range(len(self.detections)):
            if box_index not in self.box_indices:
                if self.verbose:
                    print("tracker {} created for box {}".format(
                        trackerID, box_index))
                self.initialize_tracker(trackerID, box_index)
                return
        if self.verbose:
            print("tracker {} not created".format(trackerID))

    def initialize_tracker(self, trackerID, box_index):
        self._reset_tracker(trackerID)
        left = self.detections[box_index][1]
        top = self.detections[box_index][0]
        right = self.detections[box_index][3]
        bot = self.detections[box_index][2]
        cx = (left + right) / 2
        cy = (top + bot) / 2
        self.particles[trackerID, :, 0] = [cx]*self.num_particles
        self.particles[trackerID, :, 1] = [cy]*self.num_particles
        self.update_tracker_this_det(
            trackerID, box_index, np.zeros(self.num_particles))

    def update_all(self, image, boxes, labels=None, verbose=False):
        self.min_allowable_likelihood = -0.5
        self.cov = 0.001
        self.img = image
        for box in boxes:
            if max(box) > 1 or min(box) < 0:
                raise ValueError
        self.detections = boxes
        self.verbose = True
        self.box_indices = set()
        self.labels = labels
        if self.verbose:
            print("\n\nStarting New Tracker Update \n")
        self.update_all_initialized_trackers()
        for trackerID in range(self.num_trackers):
            if self.initialized_trackers[trackerID] == 0:
                self.try_to_start_tracking(trackerID)
        return self.get_boxes()
