# not really sure the best way to test besides just visualizing some
# simulations and seeing if they make sense.
# TODO update and validate.

import numpy as np
import sys

sys.path.append("..")
import state_history
import scene


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
STATE = state_history.StateHistory()

im_h, im_w = (480, 640)
left, right, top, bot = (245, 395, 300, 400)
box = (left, right, top, bot)
STATE.update_state(box, im_h, im_w, args, test=True, object_key=1)

left, right, top, bot = (245, 395, 310, 410)
box = (left, right, top, bot)
STATE.update_state(box, im_h, im_w, args, test=True, object_key=1)

STATE.set_ego_speed(25)
this_scene = scene.Scene(STATE.states, ego_speed=(0,STATE.get_ego_speed()), ego_accel=(0,0))
rollouts = this_scene.simulate(1,   # n_sims
                               50, # H
                               0.1, # step
                               False)# verbose
from matplotlib import pyplot as plt

for path in rollouts:
    t = 0
    for curr_scene in path:
        t += 0.1
        for veh_id in curr_scene.keys():
            new_pos_x = curr_scene[veh_id].rel_x
            new_pos_y = curr_scene[veh_id].rel_y
            if veh_id == "ego":
                plt.plot(new_pos_x, new_pos_y, 'go', label=veh_id)
            else:
                plt.plot(new_pos_x, new_pos_y, 'bo', label=veh_id)
            print(veh_id, new_pos_x, new_pos_y)
        plt.legend()
        plt.title("{0:.2f}".format(t))
        plt.ylim(-100,100)
        plt.xlim(-100,100)
        plt.savefig("tests/plots/T: {0:.2f}.png".format(t))
        plt.close()
