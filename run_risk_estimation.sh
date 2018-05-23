# Just a small script to run the webcam.py script by moving into the proper directory

# Be in the virtualenv b
#source `which virtualenvwrapper.sh`
#workon tf-py3

YOLO=true
YOLO=false #comment to use yolo model

START_LOC=$(pwd)

SOURCE=${START_LOC}/videos/kitti_5s.mp4
SOURCE=${START_LOC}/videos/test_1.avi
#SOURCE=${START_LOC}/videos/Untitled2.mov

#SOURCE=0
#SOURCE=1

SAVE='true'
#SAVE='false'
SAVE_PATH='/home/derek/driver_risk_estimation_mono_video/video_yolo_'${YOLO}'.mp4'


# TODO THIS IS WHERE I CHANGE THINGS FOR GETTING RAW VIDEO
RUN='4b'
SOURCE='/scratch/derek/video_captures/video'${RUN}'.mp4'
SAVE_PATH='/scratch/derek/video_captures/video'${RUN}'_marked.mp4'


QUEUE=1
DO_TRACK='true'
TRACK_REFRESH=5
DET_THRESH=0.01      # above 1 means nothing will get marked.
ACCEPT_SPEED='false' # enter ego vehicle speed (currently mph)

FOCAL=350
CAR_WIDTH=1.8               # meters
CAMERA_HEIGHT=1.15          # meters
MIN_CAMERA_ANGLE=54.5       # degrees
MAX_CAMERA_ANGLE_HORIZ=90.0 # degrees, aka FOV
RELATIVE_HORIZON=0.45       # between 0 and 1

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
    cd /home/derek/env_tf_models_research/object_detection

    python $(echo $START_LOC)/run_risk_estimation.py \
        --source $SOURCE \
        --save $SAVE --save_path $SAVE_PATH \
        --queue $QUEUE --focal $FOCAL --carW $CAR_WIDTH \
        --det_thresh $DET_THRESH --cameraH $CAMERA_HEIGHT \
        --cameraMinAngle $MIN_CAMERA_ANGLE --horizon $RELATIVE_HORIZON \
        --cameraMaxHorizAngle $MAX_CAMERA_ANGLE_HORIZ \
        --track $DO_TRACK --tracker_refresh $TRACK_REFRESH \
        --accept_speed $ACCEPT_SPEED \
        --extra_import_path $START_LOC
    cd $START_LOC

    #deactivate 
fi

