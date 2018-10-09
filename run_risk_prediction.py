import numpy as np
import os
import sys
import tensorflow as tf
import cv2
import time

from collections import defaultdict
from io import StringIO

# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("..")
from object_detection.utils import ops as utils_ops
#session_config.gpu_options.per_process_gpu_memory_fraction = 0.6
from utils import label_map_util
from utils import visualization_utils as vis_util

# Now importing custom packages
# We want to add the directory that contains this file to the path list:
sys.path.append(os.path.dirname(__file__))

import obj_det_state
import risk_pred
import tracker
from driver_risk_utils import argument_utils, display_utils, general_utils, gps_utils


class launcher:

    def __init__(self, args):
        self.save_video = args.save
        self.save_path = args.save_path
        self.model_name = args.model
        self.path_to_checkpoint = self.model_name + '/frozen_inference_graph.pb'
        self.path_to_labels = args.labels
        if 'kitti' in self.path_to_labels:
            self.num_classes = 2
        else: #Coco?
            self.num_classes = 90
        self.all_args = args # make a copy for future use to avoid always referencing.

        self.gps_interface = None
        if args.use_gps:
            self.gps_interface = gps_utils.gps_interface(self.all_args.gps_source)

        self.state = obj_det_state.state()
        self.state.set_ego_speed_mph(35)

        self.risk_predictor = risk_pred.risk_predictor(
            args.risk_H,
            step=args.risk_step,
            col_x=args.col_tol_x,
            col_y=args.col_tol_y,
            ttc_tolerance=args.ttc_tol
        )
        ####

        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
          od_graph_def = tf.GraphDef()
          with tf.gfile.GFile(self.path_to_checkpoint, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')

        label_map = label_map_util.load_labelmap(self.path_to_labels)
        categories = label_map_util.convert_label_map_to_categories(
                label_map,
                max_num_classes=self.num_classes,
                use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)

        self.all_args = args


    def launch(self):
        with tf.device('/gpu:0'): # TODO make device an argument
         with self.detection_graph.as_default():
          with tf.Session() as sess:
            one_time_runner = runner(self, sess)
            one_time_runner.run()

class runner:
    # this class is tasked with just running one instantiation.
    # it is called after tf.Session()
    def __init__(self, launcher, sess):
        self.launcher = launcher
        self.sess = sess
        self.reset_vars()

    def reset_vars(self):
        # reset some tracking variables used throughout the class.
        # Buffers to allow for batch demo
        self.buffer_inp = list()
        self.buffer_pre = list()
        self.elapsed = 0
        self.start = time.time()
        self.fps = 1
        self.done = False # updated in process_frame()

    def run(self):
        self.reset_vars()
        self.init_camera(self.launcher.all_args.source)
        if self.launcher.all_args.save:
            self.init_video_write()

        self.framework(self.sess)
        self.image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')

        # Tracker
        self.tracker_obj = tracker.tracker(
            self.launcher.all_args, "KCF", self.height, self.width, self.launcher.category_index)

        if self.launcher.all_args.accept_speed:
            print("Press 's' to enter speed.")
        while self.camera.isOpened() and not self.done:
            self.process_frame()

        end = time.time()
        print("Total time: ", end - self.start)
        print("Frames processed: ", self.elapsed)
        print("Average FPS: ", self.elapsed/(end - self.start))
        if self.launcher.all_args.save:
            self.videoWriter.release()
        self.camera.release()
        if self.using_camera:
            cv2.destroyAllWindows()

    def framework(self, sess):
        # Get handles to input and output tensors
        ops = tf.get_default_graph().get_operations()
        all_tensor_names = {output.name for op in ops for output in op.outputs}
        self.tensor_dict = {}
        for key in ['num_detections', 'detection_boxes', 'detection_scores',
                    'detection_classes']:
            tensor_name = key + ':0'
            if tensor_name in all_tensor_names:
                self.tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(tensor_name)

    # source of 0 for webcam 0, 1 for webcam 1, a string for an actual file
    def init_camera(self, INPUT_FILE):
        self.using_camera = (type(INPUT_FILE) is not str)
        if not self.using_camera:
            assert os.path.isfile(INPUT_FILE), \
                    'file {} does not exist'.format(INPUT_FILE)
        else:
            print('Press [ESC] to quit demo')
        self.camera = cv2.VideoCapture(INPUT_FILE)
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
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        if self.using_camera:
            fps=FPS
        else:
            fps = round(self.camera.get(cv2.CAP_PROP_FPS))
        self.videoWriter = cv2.VideoWriter(
                self.save_path, fourcc, fps, (self.width, self.height))

    def process_frame(self):
        self.elapsed += 1
        self.tracker_obj.update_if_init(self.elapsed)
        self.fps = general_utils.get_fps(self.start, self.elapsed)

        self.tracker_obj.check_and_reset_multitracker(self.launcher.state)

        _, image_np = self.camera.read()
        if image_np is None:
            print('\nEnd of Video')
            self.done = True
            return

        #image_np_expanded = np.expand_dims(image_np, axis=0)
        im_height, im_width, _ = image_np.shape
        self.buffer_inp.append(image_np)
        self.buffer_pre.append(image_np)
        self.tracker_obj.update_im_shape(im_height, im_width)

        if self.elapsed % self.launcher.all_args.queue == 0:
            net_out = None
            if self.tracker_obj.should_reset():
                net_out = self.sess.run(self.tensor_dict,
                                        feed_dict={self.image_tensor: self.buffer_pre})
            for i in range(self.launcher.all_args.queue):
                # Visualization of the results of a detection
                do_convert = True
                if self.tracker_obj.use_tracker:
                    boxes, do_convert, labels = self.tracker_obj.update_one(i, net_out, self.buffer_inp)
                else:
                    boxes = net_out['detection_boxes'][i][np.where(\
                            net_out['detection_scores'][i] >= self.launcher.all_args.det_thresh)]
                    labels = [self.launcher.category_index[key]['name'] for key in \
                            net_out['detection_classes'][i][np.where(\
                                net_out['detection_scores'][i] >= self.launcher.all_args.det_thresh)]
                            ]
                risk = self.launcher.risk_predictor.prev_risk
                calculate_risk = self.elapsed % self.launcher.all_args.calc_risk_n == 1
                if calculate_risk:
                    risk = self.launcher.risk_predictor.get_risk(self.launcher.state, risk_type="online", n_sims=50, verbose=False)
                img = display_utils.display(
                        self.launcher.all_args,
                        self.launcher.state,
                        risk,
                        self.buffer_inp[i],
                        boxes,
                        do_convert,
                        labels,
                        fps=self.fps
                    )
                if self.launcher.all_args.save:
                    self.videoWriter.write(img)
                cv2.imshow('', img)
            self.buffer_inp = list()
            self.buffer_pre = list()
        if self.launcher.all_args.use_gps:
            self.launcher.state.set_ego_speed(self.launcher.gps_interface.get_reading())
        choice = cv2.waitKey(1)
        if choice == 27:
            self.done = True
            return
        elif self.launcher.all_args.accept_speed and choice == ord('s'):
            speed = input("Enter speed (mph): ")
            try:
                self.launcher.state.set_ego_speed_mph(float(speed))
            except ValueError:
                print("Please enter a float or integer.")


if __name__ == "__main__":
    args = argument_utils.parse_args()
    launcher_obj = launcher(args)
    launcher_obj.launch()
