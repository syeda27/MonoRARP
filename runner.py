import numpy as np
import os
import sys
import tensorflow as tf
import cv2
import time

from collections import defaultdict

# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("..")
from object_detection.utils import ops as utils_ops
#session_config.gpu_options.per_process_gpu_memory_fraction = 0.6
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

# Now importing custom packages
# We want to add the directory that contains this file to the path list:
sys.path.append(os.path.dirname(__file__))

import state_history
import risk_predictor
import embedded_risk_predictor
import tracker
import display
from driver_risk_utils import argument_utils, general_utils, gps_utils


class Runner:
    """
    This class is tasked with just running one instantiation.
    it is called after tf.Session() is initialized, and is where the interesting
    processing happens.
    Notably, this initializes the object detection state, risk predictor, etc.

    Important functions are:
      run() -- the main function for a `runner`. Tracker is initialized here.
      process_frame() and process_queue()
      init_camera() and init_video_write()
      get_detected_objects() and get_risk()
    """
    def __init__(self, launcher, sess):
        """
        Arguments
          launcher: a launcher object that contains some key components, like the arguments.
          sess: a TF session object.
        """
        self.launcher = launcher
        self.sess = sess
        self.gps_interface = None
        if launcher.all_args.use_gps:
            self.gps_interface = gps_utils.GPS_Interface(launcher.all_args.gps_source)
        self.thread_queue_size = launcher.thread_queue_size

        self.state = state_history.StateHistory()
        self.state.set_ego_speed_mph(35)

        risk_constructor = risk_predictor.RiskPredictor
        if launcher.all_args.embedded_risk:
            risk_constructor = embedded_risk_predictor.EmbeddedRiskPredictor

        self.risk_predictor = risk_constructor(
            sim_horizon=launcher.all_args.risk_H,
            sim_step=launcher.all_args.risk_step,
            ttc_horizon=launcher.all_args.ttc_H,
            ttc_step=launcher.all_args.ttc_step,
            collision_tolerance_x=launcher.all_args.col_tol_x,
            collision_tolerance_y=launcher.all_args.col_tol_y,
            max_threads=launcher.all_args.max_risk_threads
        )
        self.reset_vars()

    def reset_vars(self):
        """
        reset some tracking variables used throughout the class.
        """
        # Buffers to allow for batch demo
        self.input_buffer = list()
        self.elapsed = 0
        self.start = time.time()
        self.fps = 1
        self.done = False # updated in process_frame()
        self.risk_predictor.reset()
        self.start_loop = time.time()

    def init_camera(self, input):
        """
        Initializes everything needed to use the camera, if needed.
        Othewise, sets up cv2.VideoCapture from the input file.

        Sets internal variables for:
          using_camera: whether the input is a camera or not
            - if it is not a camera, then it is a file.
          camera: the actual capture device returneed by cv2.VideoCapture()
          self.height: the height of the capture device's frame.
          self.width: the width of the capture device's frame.

        Arguments
          input: a string for the file to load from, or an int to specificy
            which input camera source to use.

        Raises
          AssertionError if any of the below:
            (1) input is a file_path that does not exist.
            (2) opening the camera failed.
        """
        self.using_camera = (type(input) is not str)
        if not self.using_camera:
            assert os.path.isfile(input), \
                    'file {} does not exist'.format(input)
        else:
            print('Press [ESC] to quit demo')
        self.camera = cv2.VideoCapture(input)
        assert self.camera.isOpened(), \
                'Cannot capture source'
        if self.using_camera:
            cv2.namedWindow('', 0)
            _, frame = self.camera.read()
            self.height, self.width, _ = frame.shape
            cv2.resizeWindow('', self.width, self.height)
        else:
            _, frame = self.camera.read()
            self.height, self.width, _ = frame.shape

    def init_video_write(self, FPS=6):
        """
        This function initializes the video writer. It should only be called
        if it has been determined through the launcher args that we want to save
        the output video (with all information overlaid onto it) to a file.

        Sets internal variables for:
          videoWriter: The object used to write the video, frame by frame.

        Arguments
          FPS: the target frames per second for the video writer.
        """
        # TODO better testing of FPS to make sure nothing breaks or anything like that
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        if self.using_camera:
            fps=FPS
        else:
            fps = round(self.camera.get(cv2.CAP_PROP_FPS))
        self.videoWriter = cv2.VideoWriter(
                self.launcher.save_path,
                fourcc,
                fps,
                (self.width, self.height))

    def framework(self):
        """
        Get handles to input and output tensors.

        Sets internal variables for:
          tensor_dict: Keeps track of all tensor names for the important
            (and currently hardcoded) outputs tensors.
            This is very important for getting the output from the object
            detection network.

        """
        ops = tf.get_default_graph().get_operations()
        all_tensor_names = {output.name for op in ops for output in op.outputs}
        self.tensor_dict = {}
        for key in ['num_detections', 'detection_boxes', 'detection_scores',
                    'detection_classes']:
            tensor_name = key + ':0'
            if tensor_name in all_tensor_names:
                self.tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(tensor_name)

    def read_image(self):
        """
        Read an image from the camera into the internal variables.

        Updates internal variables for:
          input_buffer: appends this image.
          tracker_obj: updates the shape based on this image.
          done: Set to true when the video ends (only triggered on file sources)
        """
        _, image_np = self.camera.read()
        if image_np is None:
            print('\nEnd of Video')
            self.done = True
            return

        #image_np_expanded = np.expand_dims(image_np, axis=0)
        self.input_buffer.append(image_np)

    def get_detected_objects(self, i, net_out, image):
        """
        Gets the detected objects either from the tracker or the network output,
        depending on whether or not we are re-initializing the tracker.

        Arguments
          i: the index (int) within the queue that this funciton will process.
            Important for net_out
          net_out: the dictionary containing the outputs from the object
            detection network.
            Will be None if we are not resetting the tracker.
          image: the image to process.

        Returns
          boxes: list of the detected object bounding boxes in this image.
            Will be in absolute pixel value coordinates.
          labels: list of the corresponding class labels for each box.
        """
        # TODO the use of net_out can get somewhat confusing. That means it can
        # be optimized somehow...

        if self.tracker_obj.use_tracker:
            boxes, labels = self.tracker_obj.update_one(i, net_out, image)
        else:
            boxes = net_out['detection_boxes'][i][np.where(\
                    net_out['detection_scores'][i] >= self.launcher.all_args.det_thresh)]
            labels = [self.launcher.category_index[key]['name'] for key in \
                    net_out['detection_classes'][i][np.where(\
                        net_out['detection_scores'][i] >= self.launcher.all_args.det_thresh)]
                    ]
            im_h, im_w, _ = image.shape
            for box_index, box in enumerate(boxes):
                boxes[box_index] = general_utils.convert(im_h, im_w, box)

        if len(labels) < len(boxes):
            labels.extend([""] * (len(boxes) - len(labels)))

        return boxes, labels

    def get_risk(self,
                 risk_type="online",
                 n_sims=50,
                 verbose=False):
        """
        If we want to calculate the risk this frame, we make a wrapper around
        the risk predictor. Otherwise we return the previous risk seen.

        Arguments:
          risk_type:
            String, indicate which method to use to calculate the risk.
          n_sims:
            Integer, if using the `online` method, determins how many sets of
              rollouts to simulate.
          verbose:
            Boolean, passed to called functions on whether to log.


        Returns:
          risk: a float as returned by self.risk_predictor.get_risk() indicating
            the predicted danger towards the driver.
        """
        calculate_risk = self.elapsed % self.launcher.all_args.calc_risk_n == 1
        if calculate_risk:
            return self.risk_predictor.get_risk(
                    self.state, risk_type, n_sims, verbose, self.timer)
        return self.risk_predictor.prev_risk

    def update_state(self, labels, boxes, im_h, im_w, frame_time):
        """
        Pulled out from display utils. Goes through and updates the state
        based on the boxes and labels.
        """
        for i, b in enumerate(boxes):
            (left, right, top, bot) = b
            aspect_ratio_off = general_utils.check_aspect_ratio(b)
            if labels[i] != "car" or aspect_ratio_off:
                continue
            self.state.update_state(
                (left, right, top, bot),
                im_h, im_w,
                self.launcher.all_args,
                object_key=i,
                time=frame_time)

    def visualize_one_image(self, net_out, i, frame_time):
        # Visualization of the results of a detection, not thread safe.
        if self.timer:
            self.timer.update_start("DetectObjects")
        boxes, labels = self.get_detected_objects(i, net_out, self.input_buffer[i])
        im_h, im_w, _ = self.input_buffer[i].shape
        self.update_state(labels, boxes, im_h, im_w, frame_time)
        if self.timer:
            self.timer.update_end("DetectObjects", len(boxes))
            self.timer.update_start("GetRisk")

        risk = self.get_risk()

        if self.timer:
            self.timer.update_end("GetRisk", 1)
            self.timer.update_start("Display")
        self.display_obj.update_image(self.input_buffer[i])
        img = self.display_obj.display_info(
                self.state.get_current_states_quantities(),
                risk,
                self.state.get_ego_speed_mph(),
                boxes,
                labels,
                fps=self.fps,
                frame_time=frame_time
            )

        if self.timer:
            self.timer.update_end("Display", 1)

        if self.launcher.all_args.save:
            self.videoWriter.write(img)
        cv2.imshow('', img)

    def process_queue(self, frame_time, profile=False):
        """
        Iterate over all images in queue to calculate and display everything.
        This function includes the call to run the object detection network,
        the detection of the objects, the risk, and the display calls.

        TODO: do one at a time, never need the buffer.

        Note: resets self.input_buffer to an empty list.
        """
        self.timer = None
        if profile:
            self.timer = general_utils.Timing()
            self.timer.update_start("AllCalls")

        self.tracker_obj.update_if_init(self.elapsed)
        self.tracker_obj.check_and_reset_multitracker(self.state)
        net_out = None
        if self.tracker_obj.should_reset():
            if self.timer:
                self.timer.update_start("NeuralNet")
            net_out = self.sess.run(self.tensor_dict,
                                    feed_dict={self.image_tensor: self.input_buffer})
            if self.timer:
                self.timer.update_end("NeuralNet", 1)

        self.visualize_one_image(net_out, 0, frame_time)

        self.input_buffer = list()
        if self.timer:
            self.timer.update_end("AllCalls")
            self.timer.print_stats()

    def process_frame(self, force_fps=0):
        """
        This is basically called as often as possible. It is the main wrapper
        function for an individual frame from the camera.
        I will spare the details, but the main functionality is to read an
        image and process the queue once it reaches the size specified in
        self.launcher.all_args.queue.
        It also handles the speed and all other user input.
        """
        self.elapsed += 1
        self.fps = general_utils.get_fps(self.start_loop, self.elapsed)
        if self.fps < force_fps:
            print("FPS too low ({}), so frame {} skipped.".format(
                self.fps,
                self.elapsed
            ))
            return

        if self.launcher.all_args.use_gps:
            self.state.set_ego_speed(self.launcher.gps_interface.get_reading())

        self.read_image()
        if self.done: return

        if self.elapsed % self.launcher.all_args.queue == 0:
            # this check is pretty deprecated. Careful.
            self.process_queue(time.time())

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

        self.framework()
        self.image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')

        # Tracker
        self.tracker_obj = tracker.Tracker(
            self.launcher.all_args, "KCF", self.height, self.width, self.launcher.category_index)
        # Display
        self.display_obj = display.Display()

        # TODO bottom 4 should be in a thread.
        if self.launcher.all_args.accept_speed:
            print("Press 's' to enter speed.")

        self.start_loop = time.time()

        while self.camera.isOpened() and not self.done:
            self.process_frame()
        self.done = True # in case camera closes, but still want to be done.

        end = time.time()
        print("Total time: ", end - self.start)
        print("Frames processed: ", self.elapsed)
        print("Average FPS: ", self.elapsed/(end - self.start))
        if self.launcher.all_args.save:
            self.videoWriter.release()
        self.camera.release()
        if self.using_camera:
            cv2.destroyAllWindows()
