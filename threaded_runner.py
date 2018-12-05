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


import queue
import threading

from runner import Runner


class ThreadedRunner(Runner):
    """
    This class is a subclass of Runner, that implements threaded versions of
    the functions.
    The motivation is to use threads to allow for maximum GPU utilization.
    Instead of waiting for the object detection, we process the image we got
    last go around.
    """

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
        cv2.imshow('', img)

        choice = cv2.waitKey(1)
        if choice == 27:
            self.done = True
            return
        elif self.launcher.all_args.accept_speed and choice == ord('s'):
            speed = input("Enter speed (mph): ")
            try:
                self.state.set_ego_speed_mph(float(speed))
            except ValueError:
                print("Please enter a float or integer.")

    def process_detections_fn(self, wait_time=0.05):
        process_detections_elapsed = 0
        while not self.done or not self.thread_queue.empty():
            # even if done, will empty queue first
            if self.thread_queue.empty():
                time.sleep(wait_time)
            else:
                (image_np, net_out, frame_time) = self.thread_queue.get()
                self.tracker_obj.update_if_init(process_detections_elapsed)
                self.tracker_obj.check_and_reset_multitracker(self.state)
                self.visualize_one_image(net_out, image_np, frame_time)
                # process image_np and net_out
                process_detections_elapsed += 1

    def spawn_threads(self, queue_len=3):
        if self.sep_obj_det:
            self.image_queue = queue.Queue(1)
            self.object_detection = threading.Thread(
                target=self.run_obj_det, name="obj_det", args=([
                    self.thread_wait_time,
                    self.thread_max_wait]), kwargs={}
            )
            self.object_detection.start()

        self.num_dropped = 0
        self.thread_queue = queue.Queue(queue_len)
        self.process_detections = threading.Thread(
            target=self.process_detections_fn, name="process_detections", args=([0.1]), kwargs={})
        self.process_detections.start()

    def block_for_queue(self, max_wait, wait_time):
        """
        Wait for the thread queue to not be full.
        If we reach the max weight time and the thread queue (which is output)
        is still full, we remove an element from it.
        """
        if not self.thread_queue.full(): return False
        start = time.time()
        while self.thread_queue.full() and time.time() - start < max_wait:
            time.sleep(wait_time)
        if self.thread_queue.full():
            self.thread_queue.get() # remove the oldest image.
            return True
        return False

    def run_obj_det(self, wait_time=0.05, max_wait=0.5, verbose=False):
        while not self.done or not self.thread_queue.empty():
            self.fps = general_utils.get_fps(self.start_loop, self.frames_ran_obj_det_on)
            start = time.time()
            while self.image_queue.empty():
                if verbose:
                    print("image queue empty", wait_time)
                if self.done: return
                if time.time() - start >= max_wait:
                    print("Error, waited too long for an image in object detection thread.")
                    return
                time.sleep(wait_time)
            # get image and frame time frome queue
            image_np, frame_time = self.image_queue.get()
            self.obj_det_on_image(image_np, frame_time, wait_time, max_wait, verbose)

    def obj_det_on_image(
            self, image_np, frame_time,
            wait_time=0.05, max_wait=0.5,
            verbose=False):
        net_out = self.sess.run(
            self.tensor_dict,
            feed_dict={self.image_tensor: [image_np]}
        )
        if verbose:
            print("net out run")
        self.frames_ran_obj_det_on += 1
        # update queue
        image_was_removed = self.block_for_queue(max_wait, wait_time)
        if image_was_removed:
            self.num_dropped += 1
            if verbose:
                print("Blocked for too long, image {} removed.".format(
                    self.elapsed - self.thread_queue_size))
        self.thread_queue.put((image_np, net_out, frame_time))


    def process_frame(self, max_wait=0.5, wait_time=0.05, verbose=False):
        """
        This is the main thread, aka not a seperately spawned thread
        """
        self.elapsed += 1
        self.highest_fps = general_utils.get_fps(self.start_loop, self.elapsed)

        _, image_np = self.camera.read()
        frame_time = time.time()
        if image_np is None:
            print('\nEnd of Video')
            self.done = True
            return

        if self.sep_obj_det:
            # put image and frame_time in image_queue
            if self.image_queue.full():
                self.image_queue.get()
            self.image_queue.put((image_np, frame_time))
        else:
            self.obj_det_on_image(image_np, frame_time, wait_time, max_wait)
            self.fps = general_utils.get_fps(self.start_loop, self.frames_ran_obj_det_on)
        self.speed_interface.update_estimates(image_np, frame_time)
        self.state.set_ego_speed(self.speed_interface.get_reading())

        time.sleep(wait_time)

    def run(self):
        """
        The main function for runner.
        Responsible for initializing the camera, the video writer, the tracker,
        and everything else.
        It prints some summary statistics, and importantly handles destuction.
        """
        self.timer = None
        self.reset_vars()
        self.init_camera(self.launcher.all_args.source)
        if self.launcher.all_args.save:
            self.init_video_write()
        self.framework()
        self.image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')
        # Tracker
        self.tracker_obj = tracker.Tracker(
            self.launcher.all_args,
            self.launcher.all_args.tracker_type,
            self.height,
            self.width,
            self.launcher.category_index)
        # Display
        self.display_obj = display.Display()

        if self.launcher.all_args.accept_speed:
            print("Press 's' to enter speed.")

        self.sep_obj_det = True
        self.spawn_threads(queue_len=self.thread_queue_size)

        self.start_loop = time.time()

        while self.camera.isOpened() and not self.done:
            self.process_frame(self.thread_max_wait, self.thread_wait_time)
        self.done = True

        #self.thread1.join()
        if self.sep_obj_det:
            self.object_detection.join()
        self.process_detections.join()

        end = time.time()
        print("Total time: ", end - self.start)
        print("Frames processed: ", self.elapsed)
        print("Obj Det Frames processed: ", self.frames_ran_obj_det_on)
        print("Highest FPS: ", self.highest_fps)
        print("Object Detection Through FPS: ", self.fps)
        print("Dropped frames: ", self.num_dropped)
        print("True FPS: ", (self.frames_ran_obj_det_on - self.num_dropped) / (end - self.start))
        if self.launcher.all_args.save:
            self.videoWriter.release()
        self.camera.release()
        if self.using_camera:
            cv2.destroyAllWindows()

# TODO update description
"""
Option A:
queue of length Q (2)

Thread 1:
while not self.done:
    read image.
    send to queue 1
    update self.done (video ends, etc.)
    wait for queue 1 to not be full

Thread 2:
while not self.done or queue not empty:
    take image from <end of> queue 1
    <empty queue 1? Need to to stay real time and not lag. Maybe do if
    queue grows too large??? See thread 5>
    run object detection
    <update tracker??. its cpu but thread 3 is already maxed...>
    send image, net_out to queue 2
    wait for queue 2 to not be full

Thread 3:
while not self.done or queue not empty
    take image, net_out from queue 2.
    <update tracker. (its cpu)>
    Get risk from image + boxes.
    send image, boxes, risk, state to queue 3
    wait for queue 3 to not be full

Thread 4 (display):
while not done or queue not empty
    get box, label, state, risk from queue 3.
    display image.
    save image if applicable. etc.

Thread 5 (queue checker):
while not done or queue not empty:
    check queues.
        If full for too long, remove the first image.

"""

"""
Option B (condensed):
queue of length Q (2)

Thread 1:
while not self.done:
    read image.
    update self.done (video ends, etc.)
    run object detection
    send image, net_out to queue 2
    wait for queue 2 to not be full
    If full for too long, remove the first image.

Thread 2:
while not self.done or queue not empty
    take image, net_out from queue 2.
    update tracker. (its cpu).
    Get risk from image + boxes.
    display image.
    save image if applicable. etc.

"""
