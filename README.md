# Real time driver risk estimation using object detection
### Integrated inputs include monocular video sources and GPS

## IV 2019
This branch (IV2019) is a snapshot of the repository when we submitted our paper to the IEEE Intelligent Vehicles Symposium 2019.
It should work, although we anticipate making a few edits in the near future.
The main (and limited) changes from the master branch at the time of submission are that we tried to limit the extraneous information present on this branch.
The paper we refer to can be found on [arxiv](https://arxiv.org/abs/1902.01293).**

## Install Instructions
* Need tensorflow/models/research/object_detection installed.
    - see https://github.com/tensorflow/models/tree/master/research/object_detection
    - We currently use just the resnet101 faster RCNN trained on the Kitti dataset
        - You will need to download the pretrained model
* Need openCV, numpy, etc. for python3
    - By the way, the repo is built using python3 because it is about time we all made the switch...
* See [Docs/Install.md](Docs/Install.md) for full instructions

## Running Guide
* If all you want to do is run it, the only file you need to care about is `run_risk_prediction.sh`
    - It is a bash script that handles numerous flags.
* See [Docs/RunningGuide.md](Docs/RunningGuide.md) for full guide / walthrough.

## File Breakdown
See [Docs/ProjectStructure.md](Docs/ProjectStructure.md)

# Improvements to make
See [Docs/FutureWork.md](Docs/FutureWork.md)


*This research was funded by The Allstate Corporation.*
