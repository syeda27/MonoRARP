import driver_models
'''
A vehicle class to encapsulate features and driver models

Everything is relative to the ego vehicle.
Assume the ego vehicle is at (0,0) position, so rel_x == x
'''
class vehicle:
    veh_id = "I do not exist muahahahah"
    rel_x = 0   # lateral distance to ego car
    rel_y = 0   # longitudinal distance to ego car
    rel_vx = 0  # relative lateral velocity
    rel_vy = 0  # relative longitudinal velocity
    rel_ax = 0  # the above 2 but for acceleration
    rel_ay = 0

    longitudinal_model = None   # will be IDM, but need parameters
    lateral_model = None        # will be MOBIL, but need parameters

    # this state dict is for this car
    def __init__(self, veh_id, state_dict,
            des_v = 120, hdwy_t = 1.5, min_gap=2.0, accel=0.3, deccel=3.0):
        self.veh_id = veh_id
        self.set_values(state_dict)
        self.longitudinal_model = driver_models.idm_model(des_v, hdwy_t, 
                min_gap, accel, deccel)

    def set_values(self, state_dict):         
        if "distance_x" in state_dict:
            self.rel_x = state_dict['distance_x']
        else:
            self.rel_x = 0
        if "distance_y" in state_dict:
            self.rel_y = state_dict['distance_y']
        else:
            self.rel_y = 0
        if "speed_x" in state_dict:
            self.rel_vx = state_dict['speed_x']
        if "speed_y" in state_dict:
            self.rel_vy = state_dict['speed_y']
        if "accel_x" in state_dict:
            self.rel_ax = state_dict['accel_x']
        if "accel_y" in state_dict:
            self.rel_ay = state_dict['accel_y']

    def get_action(self, fore_vehicle, ego_vehicle):
        lateral_accel = 0 # TODO lateral
        gap_y = fore_vehicle.rel_y - self.rel_y
        assert gap_y > 0
        vy = self.rel_vy
        if self.veh_id is not "ego":
            vy += ego_vehicle.rel_vy
        longitudinal_accel = self.longitudinal_model.propagate(\
                vy,                                 # own speed, absolute 
                gap_y,                              # fore gap
                self.rel_vy - fore_vehicle.rel_vy)  # positive when approaching
        return (lateral_accel, longitudinal_accel)

