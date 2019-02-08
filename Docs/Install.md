# MonoRARP

## Install Instructions
* Clone this repo.
* Need tensorflow/models/research/object_detection installed.
    - see https://github.com/tensorflow/models/tree/master/research/object_detection
    - We currently use the resnet101 faster RCNN trained on the Kitti dataset
        - You will need to download the pretrained model.
        - We also use the slower models as some offline comparisons, including `faster_rcnn_nas`.
        - It is worth noting that we currently only support models trained on the
        Kitti or Coco datasets.
          - A small change will be necessary for how to distinguish the label maps,
          I just have not gotten around to it yet.
* Update the `run_risk_prediction.sh` and check the `driver_risk_utils/defaults.py` to make sure the relevant path arguments are correct.
* Need openCV, numpy, etc. for python3
    - By the way, the repo is built using python3 because it is about time we all made the switch...
    - If using pip or pip3:
      - `pip install --user <package_name>`
        - `opencv-contrib-python`
        - `matplotlib`
        - `Pillow`
* For GPS:
   - sudp apt install gpsd


*This research was funded by The Allstate Corporation.*
