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


class Launcher:
    """
    Handles the args, which will eventually be a config file, and then
    calls an instance of the `runner` class, which does everything interesting.
    """

    def __init__(self, args):
        """
        Arguments
          args: a parser object from the argparse library.
        """
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
        """
        Picks the device, tensorflow graph, and creates the tf session, before
        initializing and runner a runner.

        Raises:
            ValueError if invalid threaded_runner method passed in.
        """
        with tf.device(self.all_args.device):
           with self.detection_graph.as_default():
            with tf.Session() as sess:
                if self.all_args.threaded_runner.lower() == "none":
                    runner.Runner(self, sess).run()
                elif self.all_args.threaded_runner.lower() == "b":
                    threaded_runner.ThreadedRunner(self, sess).run()
                else:
                    raise ValueError(
                        "Invalid method for threaded runner: {}".format(
                            self.all_args.threaded_runner.lower()))


if __name__ == "__main__":
    args = argument_utils.parse_args()
    launcher_obj = Launcher(args)
    launcher_obj.launch()
