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
import sys
sys.stdout.flush()
import scene

class risk_predictor:
    # TODO should the functions be moved into the class?

    prev_risk = 0       # TODO make list and smooth over time

    def __init__(self, 
                 H=5,               # seconds for simulation horizon
                 step=0.2,          # seconds to step by
                 col_x=2,           # tolerance (meters) to indicate a collision, laterally
                 col_y=2,           # tolerance (meters) to indicate a collision, longitudinally
                 ttc_tolerance=1.0  # A ttc of less than this 
                ):
        self.H = H
        self.step = step
        self.col_tolerance_x = col_x
        self.col_tolerance_y = col_y
        self.ttc_tolerance = ttc_tolerance
        self.prev_risk = 0

    def get_risk(self, state, risk_type="ttc", n_sims=10, verbose=False):
        if risk_type.lower() == "ttc":
            risk = calculate_ttc(
                    state, 
                    self.H, 
                    self.step,
                    self.col_tolerance_x, 
                    self.col_tolerance_y, 
                    verbose
                )
        if risk_type.lower() == "online":
            this_scene = scene.scene(state.states, 
                    ego_speed=(0,state.get_ego_speed()), 
                    ego_accel=(0,0)
                )
            rollouts = this_scene.simulate(
                    n_sims, 
                    self.H, 
                    self.step, 
                    verbose
                )
            risk = calculate_risk(
                    rollouts, 
                    self.col_tolerance_x, 
                    self.col_tolerance_y, 
                    self.ttc_tolerance, 
            verbose)
        self.prev_risk = (risk + self.prev_risk) / 2
        return self.prev_risk


'''
This first method  for ttc is just brute force. 

Propagate the scene forward by hundredths of a second until a colision
is detected, or the maximum horizon is reached.

Assume horizontal speed for the ego car is 0.
Assume all accelerations are 0.

collision tolerance in meteres

returns None if no collicion detected
'''
def calculate_ttc(state, H = 10, step = 0.1, col_tolerance_x=2, 
        col_tolerance_y=2, verbose=True, col_tolerance=None):
    if col_tolerance is not None:
        col_tolerance_x = col_tolerance
        col_tolerance_y = col_tolerance
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
                    abs(new_pos_x - ego_pos_x) <= col_tolerance_x) \
                    and (new_pos_y is not None and \
                    abs(new_pos_y - ego_pos_y) <= col_tolerance_y):
                if verbose:
                    print("calculate_ttc")
                    print(new_pos_x, ego_pos_x)
                    print(new_pos_y, ego_pos_y)
                    print("veh id", veh_id, "colliding in", t, "seconds")
                return t
    return None

''' the same but with vehicles ''' 
def calculate_ttc_veh(veh_dict, H = 10, step = 0.1, col_tolerance_x=2, 
        col_tolerance_y=2, verbose=True, col_tolerance=None):
    if col_tolerance is not None:
        col_tolerance_x = col_tolerance
        col_tolerance_y = col_tolerance
    t = 0
    while (t < H):
        t += step
        collision, veh_id = check_collisions(veh_dict, t, 
                col_tolerance_x, col_tolerance_y)
        if collision:
            if verbose:
                print("calculate_ttc")
                print("veh id", veh_id, "colliding in", t, "seconds")
            return t
    return None

def check_collisions(veh_dict, t, col_tol_x, col_tol_y):
    ego_pos_x = 0 # for now, assume no lateral motion. 
    ego_pos_y = veh_dict["ego"].rel_vy * t
    for veh_id in veh_dict.keys():
        if veh_id == "ego": continue
        new_pos_x = veh_dict[veh_id].rel_x + veh_dict[veh_id].rel_vx*t
        new_pos_y = veh_dict[veh_id].rel_y + veh_dict[veh_id].rel_vy*t
        if abs(new_pos_x - ego_pos_x) <= col_tol_x \
                and abs(new_pos_y - ego_pos_y) <= col_tol_y:
            return True, veh_id
    return False, "No collisions detected."
'''
Calculates automotive risk from a series of rollouts. 
'''
def calculate_risk(rollouts, tol_x, tol_y, tol_ttc, verbose=False):
    risk = 0.0
    lowest_ttc = 10000
    for path in rollouts:
        rollout_risk = 0.0
        for curr_scene in path:
            colliding, vehid = check_collisions(curr_scene, 0, tol_x, tol_y)
            if colliding:
                rollout_risk += 10
                continue
            ttc = calculate_ttc_veh(curr_scene, 2, 0.2, tol_x, tol_y, verbose)
            if ttc:
                if ttc < lowest_ttc:
                    lowest_ttc = ttc
                if ttc < tol_ttc:
                    rollout_risk += 1
        risk += rollout_risk / len(path)
    risk = risk / len(rollouts)
    if verbose:
        if risk > 0:
            print("lowest ttc:", lowest_ttc)
            print("risk:", risk)
    return risk
