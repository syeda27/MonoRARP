'''
This file works in conjunction with the STATE from obj_det_state.py.

It uses the information from the state to calculate the automotive risk in
the future.

It will be built up over time to include a variety of methods. 
Initially, it will use relative position and velocity to simply propagate 
  every car forward in time and determine if there are any collisions.
  If there are, it will return the time to the collision (TTC)

'''

import numpy as np
import obj_det_state
import sys
sys.stdout.flush()

'''
This first method  for ttc is just brute force. 

Propagate the scene forward by hundredths of a second until a colision
is detected, or the maximum horizon is reached.

Assume horizontal speed for the go car is 0.
Assume all accelerations are 0.

collision tolerance in meteres

returns None if no collicion detected
'''
def calculate_ttc(state, step = 0.01, H = 10, col_tolerance = 2):
    t = 0
    while (t < H):
        t += step
        ego_pos_x = 0 # for now, assume no lateral motion. 
        ego_pos_y = state.get_ego_speed() * t
        for veh_id in state.states.keys():
            this_state = state.states[veh_id][-1]
            new_pos_x = None
            new_pos_y = None
            if "distance_x" in this_state and "speed_x" in this_state:
                new_pos_x = this_state["distance_x"] + this_state["speed_x"]*t
            if "distance_y" in this_state and "speed_y" in this_state:
                new_pos_y = this_state["distance_y"] + this_state["speed_y"]*t

            if (new_pos_x is not None and \
                    abs(new_pos_x - ego_pos_x) <= col_tolerance) \
                    or (new_pos_y is not None and \
                    abs(new_pos_y - ego_pos_y) <= col_tolerance):
                return t
    return None
