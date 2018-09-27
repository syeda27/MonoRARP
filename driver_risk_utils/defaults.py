"""This file tracks default parameters to make it easier to update.
Many are used in argument_utils.py.

If applicable, the units are appened to the variable name
"""


## CAMERA DEFAULTS ##
FOCAL = 1000
CAMERA_HEIGHT_m = 1.0
CAMERA_MIN_VERTICAL_ANGLE_deg = 55.0
CAMERA_MAX_HORIZONTAL_ANGLE_deg = 90.0
HORIZON = 0.5                           # must be between 0 and 1

## GENERAL DEFAULTS ##
QUEUE = 1
DETECTION_THRESHOLD = 0.5
MODEL = 'faster_rcnn_resnet101_kitti_2018_01_28' 
LABELS = 'data/kitti_label_map.pbtxt'
CAR_WIDTH_m = 1.8
CALK_RISK_EVERY_N_FRAMES = 2

## GPS DEFAULTS ##
USE_GPS = "True"
GPS_SOURCE = "gps_logging.txt"
ACCEPT_SPEED = "False"

## TRACKER DEFAULTS ##
TRACKER_FORCE_REFRESH_EVERY_N_FRAMES = 25
DO_TRACKING = "True"

## GENERAL INPUT AND OUTPUT DEFAULTS ##
SOURCE = "0"
SAVE = "TRUE"
SAVE_PATH = '/home/derek/object_detection_mono_video/video.avi'
EXTRA_IMPORT_PATH = '/home/derek/object_detection_mono_video/'

