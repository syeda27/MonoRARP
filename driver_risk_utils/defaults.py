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
CALC_RISK_EVERY_N_FRAMES = 2
DEVICE = '/gpu:0'

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

## RISK DEFAULTS ##
RISK_HORIZON = 5 # seconds, not frames, for simulating with driver models
RISK_STEP = 0.25  # seconds to step by
TTC_HORIZON = 2  # seconds, for use when calculating low ttc events.
TTC_STEP = 0.5
COLLISION_TOL_X_m = 1 # what constitutes a collision?
COLLISION_TOL_Y_m = 2
