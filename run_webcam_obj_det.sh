# Just a small script to run the webcam.py script by moving into the proper directory

# Be in the virtualenv b
#source `which virtualenvwrapper.sh`
#workon tf-py3

YOLO=true
#YOLO=false

START_LOC=$(pwd)


if $YOLO; then
    cd ~/darkflow
    flow --model cfg/yolov2.cfg --load bin/yolov2.weights --demo camera --gpu 0.6 --labels cfg/coco.names
    cd $START_LOC
else
    cd /home/derek/env_tf_models_research/object_detection

    python $(echo $START_LOC)/run_webcam_obj_det.py

    cd $START_LOC

    #Deactivate the virtualenv
    #deactivate 

fi

