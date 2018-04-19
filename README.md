# Real time object detection, focus on monocular video sources.

## instructions
* Defaults to webcams.
* Currently skeleton.
* To run YOLO, change flag in .sh script
* otherwise will run the resnet tensorflow model trained on KITTI
* Both take webcam 0 as input.
* Make sure to redirect the paths in the .sh script to the appropriate locations.

## Prereqs
* Need darkflow installed
    - Also need the correct changes to net/help.py and net/build.py, see https://github.com/djp42/darkflow2
* Need tensorflow/models/research/object_detection installed

