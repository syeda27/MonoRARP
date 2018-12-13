import os
import sys
import tensorflow as tf
import time

# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("..")
#session_config.gpu_options.per_process_gpu_memory_fraction = 0.6
from object_detection.utils import label_map_util

# Now importing custom packages
# We want to add the directory that contains this file to the path list:
sys.path.append(os.path.dirname(__file__))

import runner
import threaded_runner
from driver_risk_utils import argument_utils

import queue
import threading
import object_detector

class TFObjectDetector(object_detector.ObjectDetector):
    """
    A class to help facilitate using different object detectors in the future.
    """

    def __init__(self, args):
        """
        Arguments
          args: a parser object from the argparse library.
        """
        self.all_args = args
        self.model_name = args.model
        self.path_to_checkpoint = self.model_name + '/frozen_inference_graph.pb'
        self.path_to_labels = args.labels
        if 'kitti' in self.path_to_labels:
            self.num_classes = 2
        else: #Coco?
            self.num_classes = 90

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

    def _framework(self):
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

    def launch(self, queue_in_size=1, queue_out_size=1, wait_time=0.05, max_wait=0.5,
            verbose=False):
        """
        Picks the device, tensorflow graph, and creates the tf session, before
        initializing and runner a runner.

        Raises:
            ValueError if invalid threaded_runner method passed in.
        """
        self.done = False
        self.image_in_q = queue.Queue(queue_in_size)
        self.detections_out_q = queue.Queue(queue_out_size)
        self.output_1 = threading.Event()
        self.detections = 0
        with tf.device(self.all_args.device):
           with self.detection_graph.as_default():
            with tf.Session() as sess:
                self._framework()
                self.image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')
                while not self.done:
                    start = time.time()
                    while self.image_in_q.empty():
                        if self.done:
                            return
                        if verbose:
                            print("image queue empty", wait_time)
                        if time.time() - start >= max_wait:
                            print("Error, waited too long for an image in object detection launch thread.")
                            print("{} >= {}".format(time.time() - start, max_wait))
                            return
                        time.sleep(wait_time)
                    # get image and frame time frome queue
                    image_np, frame_time = self.image_in_q.get()
                    if verbose:
                        print("Got image in launcher")
                    net_out = sess.run(
                        self.tensor_dict,
                        feed_dict={self.image_tensor: [image_np]}
                    )
                    if verbose:
                        print("net out run")
                    self.detections += 1
                    self.output_1.set()
                    if self.detections_out_q.full():
                        self.detections_out_q.get()
                    if verbose:
                        print("putting image")
                    self.detections_out_q.put((image_np, net_out, frame_time))
                    if verbose:
                        print("put image")

                print("Obj det launcher is done")
