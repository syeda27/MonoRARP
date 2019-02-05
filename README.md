# MonoRARP - Monocular Real-time Automotive Risk Prediction
#### Summary
This system is designed to take as input monocular video and output predictions of the automotive risk for the ego vehicle in real time (We see predictions at over 5 Hz for a 10 second risk horizon using common hardware).
Almost all of the parameters are configurable, including the risk prediction time horizon. For more details, please see the code and our [research paper](https://arxiv.org/abs/1902.01293).**

** For the exact code version associated with the IEEE IVS 2019 paper submission, see the [IV2019 branch](https://github.com/djp42/driver_risk_prediction_mono_video/tree/IV2019) of this repository.

## Install Instructions
* tensorflow/models/research/object_detection installed.
    - see https://github.com/tensorflow/models/tree/master/research/object_detection
    - We currently use just the resnet101 faster RCNN trained on the Kitti dataset
        - You will need to download the pretrained model
* openCV, numpy, etc. for python3
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



*This research was funded by The Allstate Corporation and conducted by the Stanford Intelligent Systems Laboratory.*

&Dagger; -- The name is a work in progress and any suggestions are appreciated
