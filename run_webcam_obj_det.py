import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile
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

#### FLAGS ####
SAVE_VIDEO = True
SAVE_PATH = '/home/derek/object_detection_mono_video/video.avi'

# What model to download.
MODEL_NAME = 'faster_rcnn_resnet101_kitti_2018_01_28' #'ssd_mobilenet_v1_coco_2017_11_17'
MODEL_FILE = MODEL_NAME + '.tar.gz'
DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'
PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'
PATH_TO_LABELS = os.path.join('data', 'kitti_label_map.pbtxt')#'mscoco_label_map.pbtxt')
NUM_CLASSES = 2
####


detection_graph = tf.Graph()
with detection_graph.as_default():
  od_graph_def = tf.GraphDef()
  with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
    serialized_graph = fid.read()
    od_graph_def.ParseFromString(serialized_graph)
    tf.import_graph_def(od_graph_def, name='')

label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# ymin, xmin, ymax, xmax  ===> left, right, top, bot
def convert(im_height, im_width, b):
    (ymin, xmin, ymax, xmax) = b
    (left, right, top, bot) = (int(xmin * im_width), int(xmax * im_width),
                              int(ymin * im_height), int(ymax * im_height))
    return (left, right, top, bot)

def display(im, boxes, do_convert=True):
    if type(im) is not np.ndarray:
                imgcv = cv2.imread(im)
    else: imgcv = im
    im_height, im_width, _ = imgcv.shape
    thick = int((im_height + im_width) // 300)
    for b in boxes:
        if do_convert:
            (left, right, top, bot) = convert(im_height, im_width, b)
        else:
            (left, right, top, bot) = b
        cv2.rectangle(imgcv,
                        (int(left), int(top)), (int(right), int(bot)),
                        0, thick)
    return imgcv

def framework(sess):
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
def init_camera(INPUT_FILE):
    using_camera = (type(INPUT_FILE) is not str)
    if not using_camera:
        assert os.path.isfile(INPUT_FILE), \
                'file {} does not exist'.format(INPUT_FILE)
    else:
        print('Press [ESC] to quit demo')
    camera = cv2.VideoCapture(INPUT_FILE)
    assert camera.isOpened(), \
            'Cannot caputure source'
    if using_camera:
        cv2.namedWindow('', 0)
        _, frame = camera.read()
        height, width, _ = frame.shape
        cv2.resizeWindow('', width, height)
    else:
        _, frame = camera.read()
        height, width, _ = frame.shape

    return camera, using_camera, height, width

def init_video_write(camera, using_camera, height, width, FPS=6):
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    if using_camera:
        fps=FPS
    else:
        fps = round(camera.get(cv2.CAP_PROP_FPS))
    videoWriter = cv2.VideoWriter(
            SAVE_PATH, fourcc, fps, (width, height))
    return videoWriter

def handle_tracker(i, tracker, net_out, buffer_inp, 
        im_height, im_width,
        init_tracker, det_threshold):
    do_convert = True
    if init_tracker:
        init_tracker = False
        boxes = net_out['detection_boxes'][i][np.where(\
                net_out['detection_scores'][i] >= det_threshold)]
        for b in boxes:
            tracker.add(cv2.TrackerKCF_create(), buffer_inp[i],\
                    convert(im_height, im_width, b))
        ok = None
    else:
        do_convert=False
        ok, boxes = tracker.update(buffer_inp[i])
    return boxes, init_tracker, do_convert, ok

def camera_fast(source=0, SaveVideo=SAVE_VIDEO, queue=1, det_threshold=0.5,
        refresh_tracker_t=25, # 1 to update every frame
        do_tracking=True
        ):
    with tf.device('/gpu:0'):
     with detection_graph.as_default():
      with tf.Session() as sess:
        camera, using_camera, height, width = init_camera(source)
        
        if SaveVideo:
            videoWriter = init_video_write(camera, using_camera, height, width)

        # Buffers to allow for batch demo
        buffer_inp = list()
        buffer_pre = list()
        elapsed = int()
        start = time.time()

        tensor_dict = framework(sess)
        image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')
        # Tracker
        if do_tracking:
            tracker = cv2.MultiTracker_create()
        else:
            tracker = None

        while camera.isOpened():
            elapsed += 1
            init_tracker = elapsed % refresh_tracker_t == 1
            if tracker and init_tracker: 
                #reinitialize all individual trackers by clearing
                tracker = cv2.MultiTracker_create()
            
            _, image_np = camera.read()
            if image_np is None:
                print('\nEnd of Video')
                break

            #image_np_expanded = np.expand_dims(image_np, axis=0)
            im_height, im_width, _ = image_np.shape
            buffer_inp.append(image_np)
            buffer_pre.append(image_np)

            if elapsed % queue == 0:
                net_out = sess.run(tensor_dict,
                                feed_dict={image_tensor: buffer_pre})
                for i in range(queue):
                    # Visualization of the results of a detection
                    do_convert = True
                    if tracker:
                        boxes, init_tracker, do_convert, _ = handle_tracker(i, 
                                tracker, net_out, buffer_inp,
                                im_height, im_width, init_tracker, 
                                det_threshold) 
                    else:
                        boxes = net_out['detection_boxes'][i][np.where(\
                                net_out['detection_scores'][i] >= det_threshold)]
                    img = display(buffer_inp[i], boxes, do_convert)
                    # TODO labels
                    if SaveVideo:
                        videoWriter.write(img)
                    cv2.imshow('', img)
                buffer_inp = list()
                buffer_pre = list()
            choice = cv2.waitKey(1)
            if choice == 27: break
        end = time.time()
        print("Total time: ", end - start)
        print("Frames processed: ", elapsed)
        print("Average FPS: ", elapsed/(end-start))
        if SaveVideo:
            videoWriter.release()
        camera.release()
        if using_camera:
            cv2.destroyAllWindows()

camera_fast(0)

