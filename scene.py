import numpy as np
import copy
import vehicle
import time

"""
The scene maintains the driver models and coordinates with the state.
It does not alter state, but rather maintains its own version for propagation

"""
class scene:
    vehicle_states = {}  # Used to initialize vehicle objects
                         # VehicleID : state information for vehicle
                                       # (distances, speeds, accels)
    scene = {}  # current scene: dict of vehicle id to vehicle objects
    ego_speed = (0,15)  # x, y for the ego vehicle, everything in
                        # a vehicle object is relative. this is not.
    ego_accel = (0,0)   # x, y

    lane_width = 3.7    # meters

    means = {}      # mean of parameters for IDM
    variances = {}  # variances of parameters for IDM

    def __init__(self, vehicle_states, ego_speed=(0,15), ego_accel=(0,0)):
        self.vehicle_states = vehicle_states
        self.reset_scene(vehicle_states, ego_speed, ego_accel)
        self.means = {
                "des_v": ego_speed[1],    # first IDM
                "hdwy_t": 1.5,
                "min_gap": 2.0,
                "accel": 0.5,
                "deccel": 3.0,  # below are mobil
                "p": 0.2,
                "b_safe": 3.0,
                "a_thr": 0.2,
                "delta_b": 0
                }
        self.variances = {
                "des_v": 10,    # first IDM
                "hdwy_t": 0.25,
                "min_gap": 0.25,
                "accel": 0.2,
                "deccel": 0.1,  # below are mobil
                "p": 0.1,
                "b_safe": 0.2,
                "a_thr": 0.1,
                "delta_b": 0
                }

    def clear_scene(self):
        self.scene = {}

    def set_ego(self, ego_speed, ego_accel):
        self.ego_speed = ego_speed
        self.ego_accel = ego_accel
        self.scene["ego"] = vehicle.vehicle("ego", dict()) # TODO params

    def reset_scene(self, vehicle_states, ego_speed=(0,15), ego_accel=(0,0)):
        self.clear_scene()
        self.set_ego(ego_speed, ego_accel)
        for object_key in vehicle_states.keys():
            self.scene[object_key] = vehicle.vehicle(
                object_key,
                vehicle_states[object_key])

    def update_scene(self, actions, step=0.2):
        for vehid in actions.keys():
            dvx, dvy = actions[vehid]
            self.scene[vehid].rel_x += self.scene[vehid].rel_vx*step + 0.5*dvx*(step**2)
            self.scene[vehid].rel_y += self.scene[vehid].rel_vy*step + 0.5*dvy*(step**2)

            self.scene[vehid].lateral_distance = dvx*step
            self.scene[vehid].rel_vx = self.scene[vehid].rel_vx + dvx*step
            self.scene[vehid].rel_vy = self.scene[vehid].rel_vy + dvy*step

    """
    Runs N simulations using the IDM driver model

    returns paths (aka rollouts)
    paths are a list of N path objects, (length N)
    a path object is a list of a a dictionary of the vehicles in a scene
        length (H / step)
    """
    def simulate(self, N=100, H=5, step = 0.2, verbose=False, profile=False):
        paths = [] # list of N paths, which are snapshots of the scenes
        if profile:
            start = time.time()
            deepcopy_time, n_deepcopy, sim_forward_time, n_sim = 0, 0, 0, 0
            get_action_time, n_get_action, = 0, 0
            scene_update_time, n_scene_update = 0, 0

        for i in range(N):  # TODO modularize
            path = [] # a path is a list of scenes
            t = 0
            self.reset_scene(self.vehicle_states, self.ego_speed, self.ego_accel)
            for vehid in self.scene.keys():
                self.scene[vehid].longitudinal_model.randomize_parameters(
                        self.means, self.variances)
                self.scene[vehid].lateral_model.randomize_parameters(
                        self.means, self.variances)
            if profile:
                start_dc = time.time()

            path.append(copy.deepcopy(self.scene))
            if profile:
                deepcopy_time += time.time() - start_dc
                n_deepcopy += 1
                start_sim_forward = time.time()

            while t < H:  # TODO modularize
                t += step
                if profile:
                    start_get_action = time.time()

                actions = {}
                for vehid in self.scene.keys():
                    actions[vehid] = self.scene[vehid].get_action(self, step) # dvxdt, dvydt
                    if verbose:
                        print("action for", vehid, ":", actions[vehid])
                if profile:
                    get_action_time += time.time() - start_get_action
                    n_get_action += 1
                    start_scene_update = time.time()
                self.update_scene(actions, step)
                if profile:
                    scene_update_time += time.time() - start_scene_update
                    n_scene_update += 1
                    start_dc = time.time()
                path.append(copy.deepcopy(self.scene))
                if profile:
                    deepcopy_time += time.time() - start_dc
                    n_deepcopy += 1

            if profile:
                sim_forward_time += time.time() - start_sim_forward
                n_sim += 1
            paths.append(path)
        if profile:
            print("Simulating {} paths took: {}".format(N, time.time()-start))
            print("Deepcopies {} took: {}".format(n_deepcopy, deepcopy_time))
            print("SimForward {} took: {}".format(n_sim, sim_forward_time))
            print("GetAction {} took: {}".format(n_get_action, get_action_time))
            print("SceneUpdate {} took: {}".format(n_scene_update, scene_update_time))
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

    # vehicle.rel_x is negative if left
    def get_back_vehicle(self, current_scene, me, left=True, verbose=False):
        best = None
        closest_y = 1000
        for vehid in current_scene.keys():
            if vehid == me.veh_id: continue
            them = current_scene[vehid]
            dx = them.rel_x - me.rel_x
            if left:
                if dx > 0: continue
            else:
                if dx < 0: continue
            dx = abs(dx)
            if dx > self.lane_width * 0.5 and dx < self.lane_width * 1.5:
                gap = me.rel_y - them.rel_y # positive when me is in front
                if gap < closest_y and gap > 0:
                    closest_y = gap
                    best = them
        if best is None:
            lane_x = self.lane_width
            if left:
                lane_x = -lane_x
            best = vehicle.vehicle("fake_veh",
                {"speed_x": me.rel_vx,
                 "speed_y": me.rel_vy,              # same speed
                 "distance_y": me.rel_y - 1000,     # largest gap
                 "distance_x": me.rel_x - lane_x})  # in other lane
        return best

    # TODO check negation is correct
    # me is a vehicle object
    def get_back_vehicle_left(self, current_scene, me, verbose=False):
        return self.get_back_vehicle(current_scene, me, left=True, verbose=verbose)

    def get_back_vehicle_right(self, current_scene, me, verbose=False):
        return self.get_back_vehicle(current_scene, me, left=False, verbose=verbose)
