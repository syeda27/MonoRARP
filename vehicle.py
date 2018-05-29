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
    lateral_distance = 0 # to track lane change progress

    longitudinal_model = None   # will be IDM, but need parameters
    lateral_model = None        # will be MOBIL, but need parameters

    # this state dict is for this car
    def __init__(self, veh_id, state_dict,
            des_v = 120, hdwy_t = 1.5, min_gap=2.0, accel=0.3, deccel=3.0,
            p = 0.2, b_safe = 3, a_thr = 0.2, delta_b = 0):
        self.veh_id = veh_id
        self.set_values(state_dict)
        self.longitudinal_model = driver_models.idm_model(des_v, hdwy_t, 
                min_gap, accel, deccel)
        self.lateral_model = driver_models.mobil_model(p, b_safe, a_thr,
                delta_b)

    def set_values(self, state_dict):         
        if "distance_x" in state_dict:
            self.rel_x = state_dict['distance_x']
        if "distance_y" in state_dict:
            self.rel_y = state_dict['distance_y']
        if "speed_x" in state_dict:
            self.rel_vx = state_dict['speed_x']
        if "speed_y" in state_dict:
            self.rel_vy = state_dict['speed_y']
        if "accel_x" in state_dict:
            self.rel_ax = state_dict['accel_x']
        if "accel_y" in state_dict:
            self.rel_ay = state_dict['accel_y']

    def get_lateral_accel(self, fore_vehicle, scene, step=0.2):
        if self.lateral_distance > 0: # in a lane change maneuver
            if self.lateral_distance >= scene.lane_width:
                # lane change is done, stop changing lanes
                self.lateral_distance = 0
                return -(self.rel_vx + scene.ego_speed[0]) / step
            else:
                # still changing lanes
                return 0
        # see if we want to change lanes
       
        # check move right
        back_vehicle_right = scene.get_back_vehicle_right(scene.scene, self)
        backs_fore_vehicle_right = scene.get_fore_vehicle(scene.scene, back_vehicle_right)
        change_lanes = self.lateral_model.propagate(self, fore_vehicle,
                back_vehicle_right, backs_fore_vehicle_right, 
                scene.ego_speed[1], step)
        if change_lanes > 0:
            return change_lanes
        
        # check move left
        back_vehicle_left = scene.get_back_vehicle_left(\
                scene.scene, self)
        backs_fore_vehicle_left = scene.get_fore_vehicle(\
                scene.scene, back_vehicle_left)
        change_lanes = self.lateral_model.propagate(self, fore_vehicle,
                back_vehicle_left, backs_fore_vehicle_left,
                scene.ego_speed[1], step)
        return change_lanes

    '''
    wrapper around both longitudinal and lateral accel
    
    for lateral accel, we are accelerating without regard to physical 
    limitations, in order to move lanes in 1 step.
    '''
    def get_action(self, scene, step=0.2):
        fore_vehicle = scene.get_fore_vehicle(scene.scene, self)
        lateral_accel = self.get_lateral_accel(fore_vehicle, scene, step)
        gap_y = fore_vehicle.rel_y - self.rel_y
        if gap_y <= 0:
            return (0,0) # don't react
        vy = self.rel_vy + scene.ego_speed[1]
        longitudinal_accel = self.longitudinal_model.propagate(\
                vy,                                 # own speed, absolute 
                gap_y,                              # fore gap
                self.rel_vy - fore_vehicle.rel_vy)  # positive when approaching
        return (lateral_accel, longitudinal_accel)

