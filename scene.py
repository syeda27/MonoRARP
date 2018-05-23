import numpy as np
import copy
import vehicle

'''
The scene maintains the driver models and coordinates with the state.
It does not alter state, but rather maintains its own version for propagation

usage pseudocode would be, scene = scene(State[-1]) 
risk = simulate(scene, 100, H=5)
'''
class scene:
    states = {}  # Passed in, actually state.states
    scene = {}  # current scene: dict of vehicle id to vehicle objects
    ego_speed = (0,15)  # x, y for the ego vehicle, everything in
                        # a vehicle object is relative. this is not.
    ego_accel = (0,0)   # x, y

    lane_width = 3.7    # meters

    means = {}      # mean of parameters for IDM 
    variances = {}  # variances of parameters for IDM

    def __init__(self, states, ego_speed=(15,0), ego_accel=(0,0)):
        self.states = states
        self.reset_scene(states, ego_speed, ego_accel)

    def clear_scene(self):
        self.scene = {}
    
    def set_ego(self, ego_speed=(0,15), ego_accel=(0,0)):
        self.ego_speed = ego_speed
        self.ego_accel = ego_accel
        self.scene["ego"] = vehicle.vehicle("ego", dict()) # TODO params


    def reset_scene(self, states, ego_speed=(0,15), ego_accel=(0,0)):
        self.clear_scene()
        self.set_ego(ego_speed, ego_accel)
        for object_key in states.keys():
            self.scene[object_key] = vehicle.vehicle(object_key,
                states[object_key][-1])
    
    def update_scene(self, actions, step=0.2):
        for vehid in actions.keys():
            dvx, dvy = actions[vehid]
            self.scene[vehid].rel_x += self.scene[vehid].rel_vx*step + 0.5*dvx*(step**2)
            self.scene[vehid].rel_y += self.scene[vehid].rel_vy*step + 0.5*dvy*(step**2)
           
            self.scene[vehid].lateral_distance = dvx*step
            self.scene[vehid].rel_vx = self.scene[vehid].rel_vx + dvx*step
            self.scene[vehid].rel_vy = self.scene[vehid].rel_vy + dvy*step

    '''
    Runs N simulations using the IDM driver model
    
    returns paths (aka rollouts)
    paths are a list of N path objects, (length N)
    a path object is a list of a a dictionary of the vehicles in a scene
        length (H / step)
    '''
    def simulate(self, N=100, H=5, step = 0.2, verbose=False):
        paths = [] # list of N paths, which are snapshots of the scenes
        for i in range(N):
            path = [] # a path is a list of scenes
            t = 0
            self.reset_scene(self.states, self.ego_speed, self.ego_accel)
            for vehid in self.scene.keys():
                self.scene[vehid].longitudinal_model.randomize_parameters(
                        self.means, self.variances)
            path.append(copy.copy(self.scene))
            while t < H:
                t += step
                actions = {}
                for vehid in self.scene.keys():
                    actions[vehid] = self.scene[vehid].get_action(self, step) # dvxdt, dvydt
                self.update_scene(actions, step)
                path.append(copy.copy(self.scene))
            paths.append(path)
        return paths

    # me is a vehicle object
    def get_fore_vehicle(self, current_scene, me, verbose=False):
        best = None
        closest_y = 1000
        for vehid in current_scene.keys():
            if vehid == me.veh_id: continue
            them = current_scene[vehid]
            if abs(them.rel_x - me.rel_x) <= (self.lane_width / 2):
                gap = them.rel_y - me.rel_y
                if gap < closest_y and gap > 0:
                    closest_y = gap
                    best = them
        if best is None:
            best = vehicle.vehicle("fake_veh", 
                {"speed_x": me.rel_vx,
                 "speed_y": me.rel_vy,          # same speed
                 "distance_y": me.rel_y + 1000, # largest gap
                 "distance_x": me.rel_x + 10})  # unused
        if verbose:
            print("me: ", me.veh_id, "fore: ", best.veh_id)
        return best

    # me is a vehicle object
    def get_back_vehicle_left(self, current_scene, me, verbose=False): 
        best = None
        if best is None:
            best = vehicle.vehicle("fake_veh", 
                {"speed_x": me.rel_vx,
                 "speed_y": me.rel_vy, # same speed
                 "distance_y": 1000,   # largest gap
                 "distance_x": 10})    # unused
        return best

    # me is a vehicle object
    def get_back_vehicle_right(self, current_scene, me, verbose=False): 
        best = None
        if best is None:
            best = vehicle.vehicle("fake_veh", 
                {"speed_x": me.rel_vx,
                 "speed_y": me.rel_vy, # same speed
                 "distance_y": 1000,   # largest gap
                 "distance_x": 10})    # unused
        return best

