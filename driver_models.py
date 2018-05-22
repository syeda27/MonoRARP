# quick first pass at implementing IDM
# http://traffic-simulation.de/IDM.html
# http://traffic-simulation.de/MOBIL.ht
import numpy as np

# TODO MOBIL for latitudinal acceleration
class mobil_model:
    p = 0.2
    b_safe = 3
    a_thr = 0.2
    delta_b = 0

    def __init__(self, p=0.2, b_safe=3, a_thr=0.2, delta_b=0):
        self.p = p
        self.b_safe = b_safe
        self.a_thr = a_thr
        self.delta_b = delta_b
    
    '''
    Returns a boolean for whether this vehicle moving into the target lane
        would satisfy the safety criterion, as determined by our IDM model.
    We could use the back vehicle's IDM instead, but that is unknown to the
        driver and I think it is more realistic to use our own model

    this_vehicle is the vehicle we are making a decision for
    fore_vehicle is the vehicle that is currently in front of us.
    back_vehicle is the vehicle that is behind us in the target lane, 
        the one that would
        likely have to decelerate in the event of us moving over
    backs_fore_veh is the vehicle that is currently in front of
        the back_vehicle
    ego_vy is the absolute longitudinal speed that everyting is relative to.
    '''
    def main_or_something(self, this_vehicle, fore_vehicle, back_vehicle,
            backs_fore_veh, ego_vy=15):
        should_change = self.safety_criterion(this_vehicle, back_vehicle, 
                ego_vy) and \
                self.incentive_criterion(this_vehicle, back_vehicle, 
                      fore_vehicle, backs_fore_veh, ego_vy)

    def safety_criterion(self, this_vehicle, back_vehicle, ego_vy=15):
        v = back_vehicle.rel_vy + ego_vy
        new_gap = this_vehicle.rel_y - back_vehicle.rel_y
        new_dV = this_vehicle.rel_vy - v 
        accel_if_change = this_vehicle.longitudinal_model.propagate(v, new_gap, new_dV)
        return accel_if_change > -self.b_safe

    def incentive_criterion(self, this_vehicle, back_vehicle, fore_vehicle,
            back_vehicles_fore_vehicle, ego_vy = 15):
        back_accel_if_change = get_accel_y(this_vehicle, back_vehicle, 
                ego_vy, this_vehicle.longitudinal_model)
        back_accel_no_change = get_accel_y(back_vehicles_fore_vehicle,
                back_vehicle, ego_vy, this_vehicle.longitudinal_model)

        my_accel_if_change = get_accel_y(back_vehicles_fore_vehicle,
                this_vehicle, ego_vy, this_vehicle.longditudinal_model)
        my_accel_no_change - get_accel_y(fore_vehicle, this_vehicle, 
                ego_vy, this_vehicle.longitudinal_model)
        
        return (my_accel_if_change - my_accel_no_change > self.p * (back_accel_no_change - back_accel_if_change) + self.a_thr)

'''
wrapper around IDM, gets longitudinal accel for back_vehicle
idm_model allows choice between using back or fore vehicle driver model
'''
def get_accel_y(fore_veh, back_veh, ego_vy, idm_model):
    v = back_veh.rel_vy + ego_vy
    gap = fore_veh.rel_y - back_veh.rel_y
    dV = back_veh.rel_vy - fore_veh.rel_vy # positive -> back veh approachin
    return idm_model.propagate(v, gap, dv)

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
