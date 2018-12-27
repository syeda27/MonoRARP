"""
This file implements the ThreadedRunner class, which supports (ideally) a couple
different methods for threading the DRIVR system.

Method A:
Highly decomposed threading.
NOT IMPLEMENTED

Method B:
Use the main thread for GPU object detection, and have a queue to communicate
with the thread doing tracker, risk, display, etc.

"""

import numpy as np
import os
import sys
import tensorflow as tf
import cv2
import time

# Now importing custom packages
# We want to add the directory that contains this file to the path list:
sys.path.append(os.path.dirname(__file__))

import tracker
import display
from driver_risk_utils import argument_utils, general_utils, gps_utils
from object_detectors import tensorflow_obj_det_api

import queue
import threading
from collections import defaultdict


from runner import Runner


class ThreadedRunner(Runner):
    """
    This class is a subclass of Runner, that implements threaded versions of
    the functions.
    The motivation is to use threads to allow for maximum GPU utilization.
    Instead of waiting for the object detection, we process the image we got
    last go around.
    """

    def set_done(self):
        self.done = True
        self.object_detector.done = True

    def visualize_one_image(self, net_out, image, frame_time):
        # Visualization of the results of a detection

        boxes_with_labels = self.get_detected_objects(0, net_out, image)
        im_h, im_w, _ = image.shape

        self.update_state(boxes_with_labels, im_h, im_w, frame_time)

        risk = self.get_risk()

        self.display_obj.update_image(image)
        img = self.display_obj.display_info(
                self.state.get_current_states_quantities(),
                risk,
                self.state.get_ego_speed_mph(),
                boxes_with_labels,
                fps=self.fps,
                frame_time=frame_time
            )

        if self.launcher.all_args.save:
            self.videoWriter.write(img)
        if not self.has_displayed:
            cv2.namedWindow('image',cv2.WINDOW_NORMAL)
            cv2.resizeWindow('image', 1280,720)
            self.has_displayed = True
        cv2.imshow('image', img)

        choice = cv2.waitKey(1)
        if choice == 27:
            self.set_done()
            return
        elif self.launcher.all_args.accept_speed and choice == ord('s'):
            speed = input("Enter speed (mph): ")
            try:
                self.state.set_ego_speed_mph(float(speed))
            except ValueError:
                print("Please enter a float or integer.")

    def process_detections_fn(self, wait_time=0.05, max_wait_time=1.0, verbose=False):
        self.process_detections_elapsed = 0
        this_wait_time = max_wait_time*10
        time.sleep(max_wait_time*2)
        while not self.done or not self.object_detector.detections_out_q.empty():
            # even if done, will empty queue first
            try:
                (image_np, net_out, frame_time) = self.object_detector.detections_out_q.get(
                    True, # do block
                    this_wait_time # max time to wait
                )
                this_wait_time = max_wait_time
            except queue.Empty:
                self.set_done()
                print("ERROR: Waited too long for an object detection")
                if self.process_detections_elapsed == 0:
                    print("INFO: Never got a detection")
                return
            self.tracker_obj.update_if_init(self.process_detections_elapsed)
            self.tracker_obj.check_and_reset_multitracker(self.state)
            self.visualize_one_image(net_out, image_np, frame_time)
            # process image_np and net_out
            if verbose:
                print("INFO: successfully processed an image")
            self.process_detections_elapsed += 1
            start = time.time()

    def spawn_threads(self, queue_len=3, verbose=False):
        self.num_dropped = defaultdict(int)
        self.image_queue = queue.Queue(1)

        self.obj_det_thread = threading.Thread(
            target=self.object_detector.launch,
            name="launch obj det",
            args=([
                queue_len,
                queue_len,
                self.thread_wait_time,
                self.thread_max_wait,
                verbose])
        )

        self.object_detection = threading.Thread(
            target=self.handle_obj_det,
            name="obj_det",
            args=([
                self.thread_wait_time,
                self.thread_max_wait,
                verbose])
        )

        self.process_detections = threading.Thread(
            target=self.process_detections_fn,
            name="process_detections",
            args=([
                self.thread_wait_time,
                self.thread_max_wait,
                verbose])
        )

        self.obj_det_thread.start()
        self.object_detection.start()
        self.process_detections.start()

    def handle_obj_det(self, wait_time=0.05, max_wait=0.5, verbose=False):
        # continuously run a loop to pass images to the object detector
        # Reads from image_queue
        # Writes to object_detector.image_in_q
        while not self.done:
            try:
                image_np, frame_time = self.image_queue.get(
                    True, # block
                    max_wait, # timeout
                )
                if self.object_detector.image_in_q.full():
                    self.object_detector.image_in_q.get()
                    self.num_dropped["Object_Detector_In"] += 1
                self.object_detector.image_in_q.put((image_np, frame_time))
            except queue.Empty:
                if verbose:
                    print("WARNING: Image queue was empty for too long")


    def wait_for_obj_det(self, max_wait=0.5):
        start = time.time()
        if self.elapsed <= 1:
            wait_succeed = self.object_detector.output_1.wait(max_wait*5)
        else:
            wait_succeed = self.object_detector.output_1.wait(max_wait)
        if wait_succeed:
            self.object_detector.output_1.clear()
        else:
            print("WARNING, waited too long for obj detections in wait_for_obj_det")
            print("{} > {}".format(time.time() - start, max_wait))

    def process_frame(self, max_wait=0.5, wait_time=0.05, verbose=False):
        """
        This is the main thread, aka not a seperately spawned thread
        - Writes to image_queue
        - runs speed estimator (if not self.sep_speed_est, wait for object detector)

        """
        self.elapsed += 1

        image_np = self.get_image()
        frame_time = time.time()
        if self.done:
            self.set_done()
            return # if no read image

        if self.image_queue.full():
            self.image_queue.get()
            self.num_dropped["Image Queue"] += 1
        self.image_queue.put((image_np, frame_time))
        if not self.sep_speed_est:
            # wait for object detector to complete one pass
            self.wait_for_obj_det(max_wait*2)
        else:
            if self.elapsed == 1:
                time.sleep(max_wait*2)
        self.speed_interface.update_estimates(image_np, frame_time)
        if verbose:
            print("INFO: Updated speed.")
        self.state.set_ego_speed(self.speed_interface.get_reading())
        time.sleep(wait_time/2)

    def print_more_timings(self):
        end = time.time()
        string = "\n====== Aggregates 2 ======"
        string += "\nDropped frames: " + str(self.num_dropped)
        string += "\nInput FPS: " + str(self.elapsed / (end - self.start))
        string += "\nProceseded FPS: " + str(self.process_detections_elapsed / (end - self.start))
        string += "\n==============\n"
        print(string)

    def run(self):
        """
        The main function for runner.
        Responsible for initializing the camera, the video writer, the tracker,
        and everything else.
        It prints some summary statistics, and importantly handles destuction.
        """
        self.reset_vars()
        self.init_camera(self.launcher.all_args.source)
        if self.launcher.all_args.save:
            self.init_video_write()
        # obj detector
        self.object_detector = tensorflow_obj_det_api.TFObjectDetector(self.launcher.all_args)

        # Tracker
        self.tracker_obj = tracker.Tracker(
            self.launcher.all_args,
            self.launcher.all_args.tracker_type,
            self.height,
            self.width,
            self.object_detector.category_index)
        # Display
        self.display_obj = display.Display()
        self.has_displayed = False

        if self.launcher.all_args.accept_speed:
            print("Press 's' to enter speed.")

        self.sep_speed_est = self.using_camera
        # when live video, we can run speed calcs in separate thread than obj det.
        self.spawn_threads(queue_len=self.thread_queue_size)

        self.start_loop = time.time()

        while self.camera.isOpened() and not self.done:
            self.process_frame(self.thread_max_wait, self.thread_wait_time)
        self.set_done()

        #self.thread1.join()
        self.object_detection.join()
        self.process_detections.join()
        self.obj_det_thread.join()

        self.print_more_timings()
        self._do_end_things()

# TODO update description -- for object detector classs!
"""
Main Thread:
while not self.done:
    read image.
    update self.done (video ends, etc.)
    speed estimator
    if live video:
        overwrite the length 1 image_queue with most recent image
        let the Object detection thread run
    else (saved video file):
        run object detection on current image

Object Det Thread (Optional):
    run object detection
    send image, net_out to queue 2
    wait for queue 2 to not be full
    If full for too long, remove the first image.

Thread 2:
while not self.done or queue not empty
    take image, net_out from queue 2.
    update tracker. (its cpu).
    Get risk from image + boxes.
        - this generally involves more threads
    display image.
    save image if applicable. etc.

"""
