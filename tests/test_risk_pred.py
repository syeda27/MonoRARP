
import numpy as np
import sys

sys.path.append("..")
import obj_det_state
import risk_pred

STATE = obj_det_state.state()

assert risk_pred.calculate_ttc(STATE) is None

# for argparsing
def str2bool(v):
    return v.lower() == "true"

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--queue", type=int, default=1)
parser.add_argument("--focal", type=int, default=1000)
parser.add_argument("--carW", type=float, default=1.8)
parser.add_argument("--tracker_refresh", type=int, default=25)
# 1 to update every frame
parser.add_argument("--track", type=str2bool, default="True")
parser.add_argument("--det_thresh", type=float, default=0.5)

parser.add_argument("--accept_speed", type=str2bool, default="True")

parser.add_argument("--cameraH", type=float, default=1.0)
parser.add_argument("--cameraMinAngle", type=float, default=55.0) #degrees
parser.add_argument("--cameraMaxHorizAngle", type=float, default=90.0) #degrees
parser.add_argument("--horizon", type=float, default=0.5) # 0 - 1


args = parser.parse_args()

im_h, im_w = (480, 640)
left, right, top, bot = (245, 395, 300, 400)
box = (left, right, top, bot)
STATE.update_state(box, im_h, im_w, args, test=True, object_key=1)

left, right, top, bot = (245, 395, 310, 410)
box = (left, right, top, bot)
STATE.update_state(box, im_h, im_w, args, test=True, object_key=1)

STATE.set_ego_speed(25)

print(risk_pred.calculate_ttc(STATE))
