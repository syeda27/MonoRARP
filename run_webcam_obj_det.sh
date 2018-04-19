# Just a small script to run the webcam.py script by moving into the proper directory

# Be in the virtualenv b
#source `which virtualenvwrapper.sh`
#workon tf-py3

START_LOC=$(pwd)

cd /home/derek/env_tf_models_research/object_detection

python $(echo $START_LOC)/run_webcam_obj_det.py

cd $START_LOC

#Deactivate the virtualenv
#deactivate 
