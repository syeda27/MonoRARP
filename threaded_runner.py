"""
This file implements the ThreadedRunner class, which supports (ideally) a couple
different methods for threading the DRIVR system.

Method B:
Main thread
  - Handles the camera processing
  - calls the speed estimator
    - we want this to run as often as possible, which is why we have it in the
      outer loop.
  - send images to the object detector queue, very fast.
Object Detector
  - Runs the object detector through Queue interfaces:
    - see object_detectors/tensorflow_obj_det_api.launch()
Object Detector Handler
  - Populates object_detector's input queue whenever possible, with key task
    of keeping it at length 1 (requires emptying it often)
Process Detections
  - Reads from the object detector's output queue to start all remaining
    components of the system: tracker, state, risk, display.
  - splitting this into more threads does not increase runtime since they are all
    serial processes that fully occupy the CPU already.
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

import queue
import threading
from collections import defaultdict

import tracker
import display
from driver_risk_utils import argument_utils, general_utils, gps_utils
from object_detectors import tensorflow_obj_det_api
from runner import Runner


class ThreadedRunner(Runner):
    """
    This class is a subclass of Runner, that implements threaded versions of
    the functions.
    The motivation is to use threads to allow for maximum GPU and CPU utilization.
    Instead of waiting for the object detection, we process the image we got
    last go around, speeding up overall runtime through parallel processes.
    """

    def __init__(self, launcher, sess=None):
        super(ThreadedRunner, self).__init__(launcher, sess)
        self.force_speed_wait = launcher.all_args.offline
        # TODO currently unused
        # if running offline, we force the speed estimator to wait.
        # There is an option add the ability to force a specific FPS from the
        # video, in which case we would need to change that behavior.

    def set_done(self):
        """
        Set the "done" flag signalling the different components to terminate cleanly.
        """
        self.done = True
        self.object_detector.done = True

    def visualize_one_image(self, net_out, image, frame_time):
        """
        Visualize the results of a detections.
        Calls most subcomponents, including get_detected_objects() to update
        the tracker, update_state(), get_risk(), and dsiplay_obj().
        Also handles videoWriter (should be handled elsewhere in the future) and
        user input.

        Arguments:
          net_out: the dictionary containing the outputs from the object
            detection network.
          image: np.array() - the image to process.
          frame_time: float - the time in seconds that the image was first processed.
            The precision should be at least milliseconds.
        """
        # Visualization of the results of a detection

        boxes_with_labels = self.get_detected_objects(0, net_out, image)
        im_h, im_w, _ = image.shape

        self.state.update_all_states(boxes_with_labels, im_h, im_w, frame_time,
            img_id=self.image_id)

        risk = self.get_risk(risk_type=self.launcher.all_args.risk_type)

        self.display_obj.update_image(image)
        img = self.display_obj.display_info(
                self.state.get_current_states_quantities(),
                risk,
                self.state.get_ego_speed_mph(),
                boxes_with_labels,
                frame_time=frame_time,
                rel_horizon=self.launcher.all_args.horizon,
                show_speed=self.launcher.all_args.show_speed
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
        """
        One of the functions that becomes a thread, this is responsible for
        reading output from the object detector thread and updating the tracker,
        then the visualize_one_image().
        This is can be thought of as the tracker thread, although it includes all
        of the state estimation, risk prediction, and display as well.

        If it waits to long, it sets everything to be done, prints an ERROR,
        and exits.
        """
        self.process_detections_elapsed = 0
        # wait longer at the beginning to let things get set up
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

    def spawn_threads(self, queue_len=3, verbose=False):
        """
        Call to create and start all of the required threads and queues.
        """
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
        """
        Continuously run a loop to pass images to the object detector
        Reads from image_queue
        Writes to object_detector.image_in_q
        """
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
        """
        Wait for the object detector to complete its first pass,
        signalling that its initialization is complete.
        print warnings if waiting too long
        """
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
        self.fps = general_utils.get_fps(self.start_loop, self.elapsed)

        image_np = self.get_image()
        frame_time = time.time()
        if not self.using_camera:
            frame_time = self.image_id / self.video_fps

        if self.done:
            self.set_done()
            return # if no read image

        if self.image_queue.full():
            self.image_queue.get()
            self.num_dropped["Image Queue"] += 1
        self.image_queue.put((image_np, frame_time)) # Add frameid here?
        if not self.sep_speed_est:
            # wait for object detector to complete one pass
            self.wait_for_obj_det(max_wait*2)
        else:
            if self.elapsed == 1:
                time.sleep(max_wait*2)
        self.speed_interface.update_estimates(image_np, frame_time)
        if verbose:
            print("INFO: Updated speed.")
        self.state.set_ego_speed(self.speed_interface.get_reading(self.image_id))
        time.sleep(wait_time/2)

    def print_more_timings(self):
        """
        Help log timing information by printing some statistics.
        """
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
