# Just a small script to run the webcam.py script by moving into the proper directory

# Be in the virtualenv b
#source `which virtualenvwrapper.sh`
#workon tf-py3

YOLO=true
YOLO=false #comment to use yolo model

START_LOC=$(pwd)

SOURCE=$(echo $START_LOC)/videos/kitti_5s.mp4
SOURCE=$(echo $START_LOC)/videos/Untitled.mov

#SOURCE=0
#SOURCE=1

SAVE='true'
SAVE='false'
SAVE_PATH='/home/derek/object_detection_mono_video/video_yolo_'$(echo $YOLO)'.avi'

QUEUE=1
FOCAL=1000
CAR_WIDTH=1.8

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

    python $(echo $START_LOC)/run_webcam_obj_det.py \
        --source $SOURCE \
        --save $SAVE --save_path $SAVE_PATH \
        --queue $QUEUE --focal $FOCAL --carW $CAR_WIDTH \
        --extra_import_path $START_LOC
    cd $START_LOC

    #deactivate 
fi

