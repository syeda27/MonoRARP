# Real time driver risk estimation using object detection
### Integrated inputs include monocular video sources and GPS
** Need to update this documentation a bit. **
## Install Instructions
* Nothing directly for this repo
* If the use of YOLO is desired, install darkflow and the correct changes to net/help.py and net/build.py.
    - see https://github.com/djp42/darkflow2
    - YOLO is currently deprecated (but can be implemented as an object_detector)
    - see https://github.com/djp42/darkflow2#using-darkflow-from-another-python-application
* Need tensorflow/models/research/object_detection installed.
    - see https://github.com/tensorflow/models/tree/master/research/object_detection
    - We currently use just the resnet101 faster RCNN trained on the Kitti dataset
        - You will need to download the pretrained model
* Need openCV, numpy, etc. for python3
    - By the way, the repo is built using python3 because it is about time we all made the switch...

* See [Docs/Install.md](Docs/Install.md) for full instructions

## Running Guide
* If all you want to do is run it, the only file you need to care about is run_risk_prediction.sh
    - It is a bash script that handles numerous flags.
* To compile and run in C++ (IN PROGRESS):
    - `./compile_cpp.sh`  <-- this cleans all of the exisiting CMake files and compiled binaries,and then runs `cmake` and `make` in the `Compile` directory.
    - Then run the binaries in the `bin` file.
* See [Docs/RunningGuide.md](Docs/RunningGuide.md) for full guide / walthrough.

## File Breakdown
See [Docs/ProjectStructure.md](Docs/ProjectStructure.md)

# Improvements to make
See [Docs/FutureWork.md](Docs/FutureWork.md)



*This research was funded by The Allstate Corporation.*
