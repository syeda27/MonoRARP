# Real time driver risk estimation using object detection
### Integrated inputs include monocular video sources and GPS

## Install Instructions
* If the use of YOLO is desired, install darkflow and the correct changes to net/help.py and net/build.py.
    - see https://github.com/djp42/darkflow2
* Need tensorflow/models/research/object_detection installed.
    - see https://github.com/tensorflow/models/tree/master/research/object_detection
    - We currently use just the resnet101 faster RCNN trained on the Kitti dataset
        - You will need to download the pretrained model
* Need openCV, numpy, etc. for python3
    - By the way, the repo is built using python3 because it is about time we all made the switch...
    - If using pip:
      - `pip install --user <package_name>`
        - `opencv-contrib-python`
        - `matplotlib`
        - `Pillow`
* For GPS:
   - sudp apt install gpsd


*This research was funded by The Allstate Corporation.*
