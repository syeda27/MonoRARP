# quick first pass at implementing IDM
# http://traffic-simulation.de/IDM.html

import numpy as np

# TODO MOBIL for latitudinal acceleration

'''
idm model does longitudinal acceleration

'''
class idm_model:
    v0 = 0
    T = 0
    s0 = 0
    a = 0
    b = 0
    delta = 1 # TODO what is this


    # set initial values, defaults given
    def __init__(self, 
            des_v = 120,    # km/h
            hdwy_t = 1.5,   # s
            min_gap = 2.0,  # m
            accel = 0.3,    # m/s^2
            deccel = 3.0):  # m/s^2
        self.v0 = des_v
        self.T = hdwy_t
        self.s0 = min_gap
        self.a = accel
        self.b = deccel

    '''
    pass in dictionary of means and variances:
        keys are parameter names
    '''
    def randomize_parameters(self, means, variances):
        return

    # calculate the acceleration given current state info
    def propagate(self, 
            v,   # own_speed
            s,   # bumper_to_bumber_fore_gap
            dV,  # relative speed (positive when approaching)
            verbose=False):
        if verbose:
            print(v, s, dV)
            print(self.s0, self.v0)
        s_star = self.s0+max(0,v*self.T + (v*dV)/(2*np.sqrt(self.a*self.b)))
        dv_dt = self.a * (1 \
                - np.power(v/self.v0, self.delta) \
                - np.power(s_star / s, 2))
        return dv_dt

