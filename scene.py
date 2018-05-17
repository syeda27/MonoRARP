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
    ego_speed = (0,15)  # x, y
    ego_accel = (0,0)   # x, y
    means = {}      # mean of parameters for IDM 
    variances = {}  # variances of parameters for IDM

    def __init__(self, states, ego_speed=(15,0), ego_accel=(0,0)):
        self.states = states
        self.ego_speed = ego_speed
        self.ego_accel = ego_accel
        self.reset_scene(states, ego_speed, ego_accel)

    def clear_scene(self):
        self.scene = {}

    def reset_scene(self, states, ego_speed=(0,15), ego_accel=(0,0)):
        self.clear_scene()
        for object_key in states.keys():
            self.scene[object_key] = vehicle.vehicle(object_key,
                states[object_key][-1])
        self.scene["ego"] = vehicle.vehicle("ego",
                {"speed_x": ego_speed[0], 
                 "speed_y": ego_speed[1],
                 "accel_x": ego_accel[0],
                 "accel_y": ego_accel[1]})

    def update_scene(self, actions, step=0.2):
        for vehid in actions.keys():
            dvx, dvy = actions[vehid]
            self.scene[vehid].rel_x += self.scene[vehid].rel_vx*step + 0.5*dvx*(step**2)
            self.scene[vehid].rel_y += self.scene[vehid].rel_vy*step + 0.5*dvy*(step**2)
            
            self.scene[vehid].rel_vx = self.scene[vehid].rel_vx + dvx*step
            self.scene[vehid].rel_vy = self.scene[vehid].rel_vy + dvy*step

    def simulate(self, N=100, H=5, step = 0.2, verbose=False):
        paths = [] # list of N paths, which are snapshots of the scenes
        for i in range(N):
            path = [] # TODO what is this
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
                    fore_veh = self.get_fore_vehicle(self.scene, vehid)
                    actions[vehid] = self.scene[vehid].get_action(fore_veh,\
                            self.scene["ego"]) # dvxdt, dvydt
                self.update_scene(actions, step)
                path.append(copy.copy(self.scene))
            paths.append(path)
        return paths

    def get_fore_vehicle(self, current_scene, object_id, lane_width=1,
            verbose=True):
        best = None
        closest_y = 1000
        me = current_scene[object_id]
        for vehid in current_scene.keys():
            if vehid == object_id: continue
            them = current_scene[vehid]
            if abs(them.rel_x - me.rel_x) <= lane_width:
                gap = them.rel_y - me.rel_y
                if gap < closest_y and gap >= 0:
                    closest_y = gap
                    best = them
        if best is None:
            best = vehicle.vehicle("fake_veh", 
                {"speed_x": me.rel_vx,
                 "speed_y": me.rel_vy, # same speed
                 "distance_y": 1000,   # largest gap
                 "distance_x": 10})    # unused
        if verbose:
            print("me: ", me.veh_id, "fore: ", best.veh_id)
        return best

