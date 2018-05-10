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
import object_det_state
import sys
sys.stdout.flush()


def calculate_ttc(state):
    return 0
