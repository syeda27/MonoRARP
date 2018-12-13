# This file is the bash script to help make running the python
# risk predictor a little more manageable to run.
# Ideally, this will eventually be transformed into a configuration
# file that is just called by the python script, because who would
# make a script to call another script, that just seems unnecessary...
# But alas, I did just that, and it works pretty well.

# Be in the virtualenv b, if you want yolo
#source `which virtualenvwrapper.sh`
#workon tf-py3

YOLO=false # true or false

START_LOC=$(pwd)
# TODO PASS IN TF_LOC AS FIRST ARGUMENT (or change the path here)
TF_LOC='/home/derek/env_tf_models_research/object_detection'
if [ "$1" != "" ]; then
    TF_LOC=$1
fi

SOURCE=${START_LOC}/videos/kitti_5s.mp4
SOURCE=${START_LOC}/videos/test_1.avi
#SOURCE=${START_LOC}/videos/Untitled2.mov

# FOR WEBCAMS:
#SOURCE=0
#SOURCE=1

SAVE='false'
#SAVE_PATH=${START_LOC}'/video_yolo_'${YOLO}'.mp4'

# TODO THIS IS WHERE I CHANGE FLAGS FOR PROCESSING PREVIOUSLY SAVED VIDEOS
RUN='11a'
FULL_HD='FullFOVandHD/' # 'FullFOVandHD/' or just empty ''
SOURCE='/scratch/derek/video_captures/'${FULL_HD}'video'${RUN}'.mp4'
#SOURCE='/scratch/derek/video_captures/video'${RUN}'.mp4'
#SOURCE='/scratch/derek/video_captures/dist_test_'${RUN}'.mp4'

SAVE_PATH='/scratch/derek/video_captures/'${RUN}'_1080p.mp4'
MODEL="/scratch/derek/obj_det_models/faster_rcnn_resnet101_kitti_2018_01_28"

DO_TRACK='true'
TRACKER_TYPE="Particle"     # KCF is opencv version. Particle for ours.
TRACK_REFRESH=150             # 1 makes no refreshing

DET_THRESH=0.01             # above 1 means nothing will get marked.
USE_GPS='false'             # use speed readings from a GPS
GPS_SOURCE='gps_logging.txt'
ACCEPT_SPEED='false'         # enter ego vehicle speed (currently mph).
                             # Speeds input by the user overwrites other readings
LANE_BASED_SPEED='false'  # currently broken

DEVICE='/gpu:0'
# TODO simplify and make threaded runner either on or off, no options
THREADED_RUNNER='B'          # The runner-level threading method, or 'None'
THREAD_QUEUE_SIZE=3          # The size of the queue for threaded_runner
THREAD_WAIT_TIME=0.02        # The minimum amount of time to block on a queue, sec.
THREAD_MAX_WAIT=0.5          # maximum amount of time to block on a queue, sec.

FOCAL=350
CAR_WIDTH=1.8                # meters
CAMERA_HEIGHT=1.15           # meters
MIN_CAMERA_ANGLE=54.5        # degrees
MAX_CAMERA_ANGLE_HORIZ=115.0 # degrees, aka FOV
RELATIVE_HORIZON=0.5         # between 0 and 1

DETECT_LANES='false'

RISK_N_SIMS=10
RISK_H=5        # seconds
RISK_STEP=0.25   # seconds
COL_TOL_X=1.0    # meters
COL_TOL_Y=1.5    # meters
TTC_H=1.0        # seconds
TTC_STEP=0.25    # seconds
RISK_THREADS=10  # max number of threads (>1 --> threaded risk calcs)
EMBEDDED_RISK='true' # boolean, whether or not to calc risk while simulating.

if ($YOLO); then
    if (($SOURCE==1) || ($SOURCE==0)); then
        $SOURCE=camera$SOURCE
    fi
    cd ~/darkflow
    cmd = 'flow --model cfg/yolov2.cfg --load bin/yolov2.weights \
        --demo $SOURCE --gpu 0.75 \
        --labels cfg/coco.names'
    if ($SAVE=='true'); then
        cmd = $(cmd)' --saveVideo'
    fi
    mv video.avi $SAVE_PATH
    cd $START_LOC
else
    JOBS=`jobs -p`
    if [ '$USE_GPS' = 'true' ]; then
        echo "Running gps command to $GPS_SOURCE"
        gpsd -S 2949 -n -N -D 5 -b /dev/ttyUSB0 &> $GPS_SOURCE &
        JOBS=`jobs -p`
        sleep 1
        # TODO if you need to, change this command
    fi
    cd $TF_LOC

    python3 $(echo $START_LOC)/launcher.py \
        --source $SOURCE --model $MODEL --device $DEVICE \
        --save $SAVE --save_path $SAVE_PATH \
        --focal $FOCAL --carW $CAR_WIDTH \
        --det_thresh $DET_THRESH --cameraH $CAMERA_HEIGHT \
        --cameraMinAngle $MIN_CAMERA_ANGLE --horizon $RELATIVE_HORIZON \
        --cameraMaxHorizAngle $MAX_CAMERA_ANGLE_HORIZ \
        --track $DO_TRACK --tracker_type $TRACKER_TYPE --tracker_refresh $TRACK_REFRESH \
        --use_gps $USE_GPS --gps_source ${START_LOC}/$GPS_SOURCE \
        --lane_based_speed $LANE_BASED_SPEED \
        --accept_speed $ACCEPT_SPEED --detect_lanes $DETECT_LANES \
        --risk_H $RISK_H --risk_step $RISK_STEP --n_risk_sims $RISK_N_SIMS \
        --ttc_H $TTC_H --ttc_step $TTC_STEP \
        --col_tol_x $COL_TOL_X --col_tol_y $COL_TOL_Y \
        --embedded_risk $EMBEDDED_RISK --max_risk_threads $RISK_THREADS \
        --threaded_runner $THREADED_RUNNER --thread_queue_size $THREAD_QUEUE_SIZE \
        --thread_max_wait $THREAD_MAX_WAIT --thread_wait_time $THREAD_WAIT_TIME
    cd $START_LOC
    for job in $JOBS
    do
        echo "killing job $job"
        kill $job
    done

    #deactivate
fi
