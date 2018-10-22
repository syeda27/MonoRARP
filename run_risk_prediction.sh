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
SAVE_PATH=${START_LOC}'/video_yolo_'${YOLO}'.mp4'

# TODO THIS IS WHERE I CHANGE FLAGS FOR PROCESSING PREVIOUSLY SAVED VIDEOS
#RUN='11a'
#FULL_HD='FullFOVandHD/' # 'FullFOVandHD/' or just empty ''
#SOURCE='/scratch/derek/video_captures/'${FULL_HD}'video'${RUN}'.mp4'
#SAVE_PATH='/scratch/derek/video_captures/'${FULL_HD}'video'${RUN}'_marked.mp4'


QUEUE=1
DO_TRACK='true'
TRACK_REFRESH=10
DET_THRESH=0.01             # above 1 means nothing will get marked.
USE_GPS='false'             # use speed readings from a GPS
GPS_SOURCE='gps_logging.txt'
ACCEPT_SPEED='false'         # enter ego vehicle speed (currently mph).
                            # Speeds input by the user overwrite the gps reading

FOCAL=350
CAR_WIDTH=1.8               # meters
CAMERA_HEIGHT=1.15          # meters
MIN_CAMERA_ANGLE=54.5       # degrees
MAX_CAMERA_ANGLE_HORIZ=115.0 # degrees, aka FOV
RELATIVE_HORIZON=0.5        # between 0 and 1

RISK_H=5      # seconds
RISK_STEP=0.2 # seconds
COL_TOL_X=2.0 # meters
COL_TOL_Y=2.0 # meters
TTC_TOL=1.0   # seconds


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

    python3 $(echo $START_LOC)/run_risk_prediction.py \
        --source $SOURCE \
        --save $SAVE --save_path $SAVE_PATH \
        --queue $QUEUE --focal $FOCAL --carW $CAR_WIDTH \
        --det_thresh $DET_THRESH --cameraH $CAMERA_HEIGHT \
        --cameraMinAngle $MIN_CAMERA_ANGLE --horizon $RELATIVE_HORIZON \
        --cameraMaxHorizAngle $MAX_CAMERA_ANGLE_HORIZ \
        --track $DO_TRACK --tracker_refresh $TRACK_REFRESH \
        --use_gps $USE_GPS --gps_source ${START_LOC}/$GPS_SOURCE \
        --risk_H $RISK_H --risk_step $RISK_STEP --ttc_tol $TTC_TOL \
        --col_tol_x $COL_TOL_X --col_tol_y $COL_TOL_Y \
        --accept_speed $ACCEPT_SPEED
    cd $START_LOC
    for job in $JOBS
    do
        echo "killing job $job"
        kill $job
    done

    #deactivate
fi
