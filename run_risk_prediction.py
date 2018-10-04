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

        # TODO move these to the self.all_args
        self.risk_predictor = risk_pred.risk_predictor(H=5, step=0.25, col_x=2, col_y=2)
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


    def framework(self, sess):
        # Get handles to input and output tensors
        ops = tf.get_default_graph().get_operations()
        all_tensor_names = {output.name for op in ops for output in op.outputs}
        tensor_dict = {}
        for key in ['num_detections', 'detection_boxes', 'detection_scores',
                    'detection_classes']:
            tensor_name = key + ':0'
            if tensor_name in all_tensor_names:
                tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(tensor_name)
        return tensor_dict

    # source of 0 for webcam 0, 1 for webcam 1, a string for an actual file
    def init_camera(self, INPUT_FILE):
        using_camera = (type(INPUT_FILE) is not str)
        if not using_camera:
            assert os.path.isfile(INPUT_FILE), \
                    'file {} does not exist'.format(INPUT_FILE)
        else:
            print('Press [ESC] to quit demo')
        camera = cv2.VideoCapture(INPUT_FILE)
        assert camera.isOpened(), \
                'Cannot capture source'
        if using_camera:
            cv2.namedWindow('', 0)
            _, frame = camera.read()
            height, width, _ = frame.shape
            cv2.resizeWindow('', width, height)
        else:
            _, frame = camera.read()
            height, width, _ = frame.shape

        return camera, using_camera, height, width

    def init_video_write(self, camera, using_camera, height, width, FPS=6):
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        if using_camera:
            fps=FPS
        else:
            fps = round(camera.get(cv2.CAP_PROP_FPS))
        videoWriter = cv2.VideoWriter(
                self.save_path, fourcc, fps, (width, height))
        return videoWriter

    def camera_fast(self):
        with tf.device('/gpu:0'): # TODO make device an argument
         with self.detection_graph.as_default():
          with tf.Session() as sess:
            camera, using_camera, height, width = self.init_camera(self.all_args.source)
            
            if self.all_args.save:
                videoWriter = self.init_video_write(camera, using_camera, height, width)
            

            # Buffers to allow for batch demo
            buffer_inp = list()
            buffer_pre = list()
            elapsed = 0
            start = time.time()
            fps = 1

            tensor_dict = self.framework(sess)
            image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')
            # Tracker
            tracker_obj = tracker.tracker(self.all_args, "KCF", height, width, 
                    self.category_index)
            
            if self.all_args.accept_speed:
                print("Press 's' to enter speed.")
            while camera.isOpened():
                elapsed += 1
                tracker_obj.update_if_init(elapsed)
                fps = general_utils.get_fps(start, elapsed)

                tracker_obj.check_and_reset_multitracker(self.state)

                _, image_np = camera.read()
                if image_np is None:
                    print('\nEnd of Video')
                    break

                #image_np_expanded = np.expand_dims(image_np, axis=0)
                im_height, im_width, _ = image_np.shape
                buffer_inp.append(image_np)
                buffer_pre.append(image_np)
                tracker_obj.update_im_shape(height, width)

                if elapsed % self.all_args.queue == 0:
                    net_out = None
                    if tracker_obj.should_reset():
                        net_out = sess.run(tensor_dict,
                                           feed_dict={image_tensor: buffer_pre})
                    for i in range(self.all_args.queue):
                        # Visualization of the results of a detection
                        do_convert = True
                        if tracker_obj.use_tracker:
                            boxes, do_convert, labels = tracker_obj.update_one(i, net_out, buffer_inp)
                        else:
                            boxes = net_out['detection_boxes'][i][np.where(\
                                    net_out['detection_scores'][i] >= self.all_args.det_thresh)]
                            labels = [self.category_index[key]['name'] for key in \
                                    net_out['detection_classes'][i][np.where(\
                                        net_out['detection_scores'][i] >= det_threshold)]
                                    ]
                        img = display_utils.display(
                                self.all_args, 
                                self.state, 
                                self.risk_predictor,
                                buffer_inp[i], 
                                boxes, 
                                do_convert, 
                                labels, 
                                fps=fps, 
                                calculate_risk = elapsed % self.all_args.calc_risk_n==1
                            )
                        if self.all_args.save:
                            videoWriter.write(img)
                        cv2.imshow('', img)
                    buffer_inp = list()
                    buffer_pre = list()
                if self.all_args.use_gps:
                    self.state.set_ego_speed(self.gps_interface.get_reading())
                choice = cv2.waitKey(1)
                if choice == 27: 
                    break
                elif self.all_args.accept_speed and choice == ord('s'):
                    speed = input("Enter speed (mph): ")
                    try:
                        self.state.set_ego_speed_mph(float(speed))
                    except ValueError:
                        print("Please enter a float or integer.")

            end = time.time()
            print("Total time: ", end - start)
            print("Frames processed: ", elapsed)
            print("Average FPS: ", elapsed/(end-start))
            if self.all_args.save:
                videoWriter.release()
            camera.release()
            if using_camera:
                cv2.destroyAllWindows()


if __name__ == "__main__":
    args = argument_utils.parse_args()
    launcher_obj = launcher(args)
    launcher_obj.camera_fast()

