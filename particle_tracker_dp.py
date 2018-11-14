
from collections import defaultdict
import particle_trackers_ar

class ParticleTrackerDP(particle_trackers_ar.ParticleTracker):

    def rank_detections(self):
        self.ranked_detections_per_tracker = defaultdict(list)
        for trackerID in self.unassigned_initialized_trackers:
            self.rank_detections_t(trackerID)

    def rank_detections_t(self, trackerID):
        # for this tracker, compute p(each detection and rank)
        self.ranked_detections_per_tracker[trackerID] = []
        for box_index in range(len(self.detections)):
            likelihood, cx, cy = self.likelihood_of_detection(trackerID, box_index)
            self.ranked_detections_per_tracker[trackerID].append(
                (likelihood, box_index))
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
        for trackerID in self.unassigned_initialized_trackers:
            if len(self.ranked_detections_per_tracker[trackerID]) == 0:
                self.increment_holding(trackerID)
                self.unassigned_initialized_trackers.remove(trackerID)
                continue
            (likelihood, box_i) = self.ranked_detections_per_tracker[trackerID][-1]
            if box_i in self.box_indices:
                try:
                    self.ranked_detections_per_tracker[trackerID].pop()
                except:
                    print(self.ranked_detections_per_tracker[trackerID])
                    print(likelihood, box_i)
                    raise ValueError
                continue
            if likelihood > highest_likelihood:
                highest_likelihood, tracker_matched, box_index = likelihood, trackerID, box_i
        if tracker_matched > 0:
            self.ranked_detections_per_tracker[tracker_matched].pop()
        return highest_likelihood, tracker_matched, box_index

    def increment_all_holding(self):
        for trackerID in self.unassigned_initialized_trackers:
            self.increment_holding(trackerID)

    def update_tracker_this_det(self, trackerID, box_index):
        x1 = self.detections[box_index][1]
        y1 = self.detections[box_index][0]
        x2 = self.detections[box_index][3]
        y2 = self.detections[box_index][2]
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        self.update_tracker(trackerID, cx, cy, box_index)

    def update_all_initialized_trackers(self):
        self.unassigned_initialized_trackers = set()
        self.ranked_detections_per_tracker = defaultdict(list)
        for trackerID in range(self.num_trackers):
            if self.initialized_trackers[trackerID] == 1:
                self.unassigned_initialized_trackers.add(trackerID)
                self.gen_rand_particles(trackerID)
                self.rank_detections_t(trackerID)
        while len(self.unassigned_initialized_trackers) > 0:
            (l, t, d) = self.get_highest_likelihood_pair()
            if t < 0: return
            if l < self.max_tracker_jump: # todo
                self.increment_all_holding()
                return
            self.update_tracker_this_det(t, d)
            self.unassigned_initialized_trackers.remove(t)

    def update_all(self, image, boxes, verbose=False):
        self.img = image
        self.detections = boxes
        self.verbose = True
        self.box_indices = set()
        self.update_all_initialized_trackers()
        for trackerID in range(self.num_trackers):
            if self.initialized_trackers[trackerID] == 0:
                self.try_to_start_tracking(trackerID)
        return self.get_boxes()
