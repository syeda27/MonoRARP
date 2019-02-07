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
from driver_risk_utils import argument_utils, general_utils, offline_utils

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
        self.offline = args.offline
        self.save_path = args.results_save_path
        self.overwrite_saves = args.overwrite_saves
        self.component_name = "OBJECT_DETECTOR"

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

    def _do_end_things(self, force=False):
        """
        call before returning from launch
        """
        string = "\n============== Ending Object Detector =============="
        if force:
            string += "\nWas forced."
        self.timer.update_end("Overall")
        string += "\nTiming:" + self.timer.print_stats(True)
        string += "\nNumber of detections: " + str(self.num_detections)
        string += "\n=============="
        print(string)

    def launch(self, queue_in_size=1, queue_out_size=1, wait_time=0.05, max_wait=0.5,
            verbose=False):
        """
        THREAD interface:
        reads from self.image_in_q
        writes to self.detections_out_q
        signals self.output_1 when a detection completes

        Picks the device, tensorflow graph, and creates the tf session, before
        entering a loop to process detections.

        Raises:
            ValueError if invalid threaded_runner method passed in.
        """
        self.done = False
        self.image_in_q = queue.Queue(queue_in_size)
        self.detections_out_q = queue.Queue(queue_out_size)
        self.output_1 = threading.Event()
        self.num_detections = 0
        self.timer = general_utils.Timing()
        self.timer.update_start("Overall")
        with tf.device(self.all_args.device):
           with self.detection_graph.as_default():
            with tf.Session() as sess:
                self._framework()
                self.image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')
                while not self.done:
                    start = time.time()
                    this_wait_max = max_wait
                    if self.num_detections == 0:
                        this_wait_max = max_wait * 10
                    while self.image_in_q.empty():
                        if self.done:
                            self._do_end_things()
                            return
                        if verbose:
                            print("INFO: Image queue empty.", wait_time)
                        if time.time() - start >= this_wait_max:
                            print("ERROR: waited too long for an image in object detection launch thread.")
                            print("{} >= {}".format(time.time() - start, this_wait_max))
                            self._do_end_things(True)
                            return
                        time.sleep(wait_time)
                    self.timer.update_start("Detecting")
                    # get image and frame time frome queue
                    image_np, frame_time = self.image_in_q.get()
                    if verbose:
                        print("INFO: Got image in launcher.")
                    net_out = sess.run(
                        self.tensor_dict,
                        feed_dict={self.image_tensor: [image_np]}
                    )
                    if verbose:
                        print("INFO: net out run.")
                    self.num_detections += 1
                    self.output_1.set()
                    if self.detections_out_q.full():
                        self.detections_out_q.get()
                    if verbose:
                        print("INFO: putting image.")
                    self.detections_out_q.put((image_np, net_out, frame_time))
                    if verbose:
                        print("INFO: put image.")
                    self.timer.update_end("Detecting")
                    if self.offline:
                        offline_utils.save_output(
                            net_out, self.component_name,
                            self.num_detections-1, self.save_path,
                            overwrite=self.overwrite_saves)

                self._do_end_things()
