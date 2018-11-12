import numpy as np
import copy
import vehicle

from driver_risk_utils import scene_utils
from driver_risk_utils import general_utils


"""
The Scene maintains the driver models and coordinates with the state.
It does not alter state, but rather maintains its own version for propagation

Maintains internal copy of the original vehicle objects (vehicle_states)
Maintains and updates internal representation of all vehicles: (scene)
        # current scene: dict of vehicle id to vehicle objects

Also maintains means and variances for the driver models we are using.
 TODO - a better way of doing this.
        means = {}      # mean of parameters for IDM and MOBIL
        variances = {}  # variances of parameters for IDM and MOBIL
"""
class Scene:
    def __init__(self,
                 vehicle_states,
                 ego_vel=(0,15),
                 ego_accel=(0,0),
                 lane_width=3.7):
        """
        Initializes the scene object with internal parameters.

        Arguments:
          vehicle_states: dict< string : vehicle_state >
            dictionary used to initialize the vehicle objects.
            VehicleID : state information for Vehicle (distance, speed, accel).
            Not updated over time, used to reset.
          ego_vel: tuple(float, float), m/s: the ego vehicle's speed. This is
            needed to initialize the ego vehicle, and adjust to absolute velocity.
          ego_accel: tuple(float, float), m/s^2: the ego vehicle's acceleration.
            This is needed to initialize the ego vehicle.
          lane_width: float, m. To represent the road and find lane changes, etc.,
            we would like an estimate of the lane width.
        """
        self.lane_width_m = lane_width
        self.vehicle_states = vehicle_states
        self.reset_scene(vehicle_states, ego_vel, ego_accel)
        self.means = {
                "des_v": ego_vel[1],    # first IDM
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
                "des_v": 5,    # first IDM
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
        """
        Clears the internal dictionary of vehicles.
        """
        self.scene = {}

    def set_ego(self, ego_vel, ego_accel):
        """
        Creates an ego vehicle for the scene based on the velocity and accel.

        Arguments:
          ego_vel: tuple(float, float), m/s: the ego vehicle's speed. This is
            needed to initialize the ego vehicle, and adjust to absolute velocity.
          ego_accel: tuple(float, float), m/s^2: the ego vehicle's acceleration.
            This is needed to initialize the ego vehicle.
        """
        self.ego_vel = ego_vel
        self.ego_accel = ego_accel
        self.scene["ego"] = vehicle.Vehicle("ego", dict()) # TODO params

    def reset_scene(self, vehicle_states=None, ego_vel=(0,15), ego_accel=(0,0)):
        """
        Wrapper to clear the scene, set the ego vehicle, and then create vehicle
          objects for each vehicle in vehicle_states.
        Arguments:
          vehicle_states: dict< string : dictionary <string:float> >
            dictionary used to initialize the vehicle objects.
            VehicleID : state information for Vehicle (distance, speed, accel).
            If None, use self.vehicle_states
          ego_vel: tuple(float, float), m/s: the ego vehicle's speed. This is
            needed to initialize the ego vehicle, and adjust to absolute velocity.
          ego_accel: tuple(float, float), m/s^2: the ego vehicle's acceleration.
            This is needed to initialize the ego vehicle.
        """
        self.clear_scene()
        self.set_ego(ego_vel, ego_accel)
        if vehicle_states == None:
            vehicle_states = self.vehicle_states
        for object_key in vehicle_states.keys():
            self.scene[object_key] = vehicle.Vehicle(
                object_key,
                vehicle_states[object_key].quantities)

    def update_scene(self, actions, step=0.2):
        """
        Propagate self.scene forward by timestep {step} using the provided actions.
        Arguments:
          actions: dict <sting : tuple<float, float> >
            A dictionary of vehicle id to acceleration lateral and acceleration
            longitudinal. Order matters because it is a tuble.
            Both values are in m/s^2.
          step: float, seconds
            The step size to take, in seconds.
        """
        for vehid in actions.keys():
            dvx, dvy = actions[vehid]
            self.scene[vehid].rel_x += self.scene[vehid].rel_vx*step + 0.5*dvx*(step**2)
            self.scene[vehid].rel_y += self.scene[vehid].rel_vy*step + 0.5*dvy*(step**2)

            self.scene[vehid].lateral_distance = dvx*step
            self.scene[vehid].rel_vx = self.scene[vehid].rel_vx + dvx*step
            self.scene[vehid].rel_vy = self.scene[vehid].rel_vy + dvy*step

    def simulate(self, N=100, H=5, step=0.2, verbose=False, timer=None):
        """
        Runs N simulations using the IDM driver model

        Arguments:
          N: integer
            The number of paths to simulate.
          H: float, seconds
            The time horizon fo which to simulate to.
          step: float, seconds
            The step size to take, in seconds.
          verbose: boolean
            Whether or not to log various occurrences.
          timer: general_utils.timing object.
            The object that is keeping track of various timing qualities.

        Returns:
          paths (aka rollouts)
            A list of N path objects, (length N)
            - a path object is a list of a dictionary of the vehicles in a
                scene, of length (H / step)
        """
        if verbose:
            print("Simulating {} paths for horizon {} by steps of {}".format(
                N, H, step))
        paths = [] # list of N paths, which are snapshots of the scenes
        if timer:
            timer.update_start("Simulating")

        for i in range(N):  # TODO modularize
            paths.append(self.simulate_one_path(H, step, verbose, timer))
        if timer:
            timer.update_end("Simulating", N)
        return paths

    def simulate_one_path(self, H, step, verbose, timer):
        """
        Simulates one path out to time horizon H by step size (step).

        Arguments:
          H: float, seconds
            The time horizon fo which to simulate to.
          step: float, seconds
            The step size to take, in seconds.
          verbose: boolean
            Whether or not to log various occurrences.
          timer: general_utils.timing object. None if profile == False
            The object that is keeping track of various timing qualities.

        Returns:
          path: A path object, which is a list of a dictionary of the vehicle
              in objects in the scene, of length (H / step).
        """
        self.reset_scene(self.vehicle_states, self.ego_vel, self.ego_accel)
        for vehid in self.scene.keys():
            self.scene[vehid].longitudinal_model.randomize_parameters(
                    self.means, self.variances)
            self.scene[vehid].lateral_model.randomize_parameters(
                    self.means, self.variances)
        path = [] # a path is a list of scenes
        if timer:
            timer.update_start("Deepcopies")
        path.append(copy.deepcopy(self.scene))
        if timer:
            timer.update_end("Deepcopies", 1)
            timer.update_start("SimForward")
        t = 0
        while t < H:  # TODO modularize
            t += step # need while loop because range doesnt handle float step.
            path.append(self.do_one_step(step, verbose, timer))

        if timer:
            timer.update_end("SimForward", 1)
        return path

    def do_one_step(self, step, verbose, timer):
        """
        Does one step of path generation.
        This involves computing the actions for each vehicle in the scene,
          and updating the scene with the appropriate actions and step size.

        Arguments:
          step: float, seconds
            The step size to take, in seconds.
          verbose: boolean
            Whether or not to log various occurrences.
          timer: general_utils.timing object.
            The object that is keeping track of various timing qualities.

        Returns:
          scene_here: dict< string : dictionary <string:float> >
            VehicleID : state information like relative position, velocity, etc.
            This variable is what is simulated, and we must create a copy
              through the use of deepcopy() to return.

        """
        if timer:
            timer.update_start("GetAction")

        actions = {}
        for vehid in self.scene.keys():
            actions[vehid] = self.scene[vehid].get_action(self, step) # dvxdt, dvydt
            if verbose:
                print("action for", vehid, ":", actions[vehid])
        if timer:
            timer.update_end("GetAction", 1)
            timer.update_start("SceneUpdate")

        self.update_scene(actions, step)
        if timer:
            timer.update_end("SceneUpdate", 1)
            timer.update_start("Deepcopies")
        scene_here = copy.deepcopy(self.scene)
        if timer:
            timer.update_end("Deepcopies", 1)
        return scene_here

    def get_fore_vehicle(self, current_scene, me, verbose=False):
        """
        Wrapper to utils to get the vehicle in front of "me". "me" is a vehicle
        object that is not necessarily the ego vehicle, just the vehicle we are
        getting the neighbor with respect to.

        Arguments:
          current_scene: dict< string : dictionary <string:float> >
            VehicleID : state information like relative position, velocity, etc.
          me: Vehicle
            A Vehicle object that exists in the current scene.
          verbose: boolean
            Whether or not to log various occurrences.

        Returns:
          A Vehicle object
        """
        return scene_utils.get_fore_vehicle(
            self.lane_width_m, current_scene, me, verbose)

    def get_back_vehicle(self, current_scene, me, left=True, verbose=False):
        """
        Wrapper to utils to get the vehicle behind "me", in either the lane to
        the left, or the lane to the right. See driver_models.py for the
        motivation for this function, specifically the MOBIL model.

        "me" is a vehicle object that is not necessarily the ego vehicle, just
        the vehicle we are getting the neighbor with respect to.

        # vehicle.rel_x is negative if left

        Arguments:
          current_scene: dict< string : dictionary <string:float> >
            VehicleID : state information like relative position, velocity, etc.
          me: Vehicle
            A Vehicle object that exists in the current scene. We are finding
            the neighboring vehicle with respect to "me"
          left: boolean
            Whether or not we want to get the vehicle behind us in
          verbose: boolean
            Whether or not to log various occurrences.

        Returns:
          A Vehicle object
        """
        return scene_utils.get_back_vehicle(
            self.lane_width_m, current_scene, me, left, verbose)

    # TODO check negation is correct
    def get_back_vehicle_left(self, current_scene, me, verbose=False):
        """
        Wrapper to get_back_vehicle, see the function comments for
        get_back_vehicle() for arguments and returns.
        """
        return self.get_back_vehicle(current_scene, me, left=True, verbose=verbose)

    def get_back_vehicle_right(self, current_scene, me, verbose=False):
        """
        Wrapper to get_back_vehicle, see the function comments for
        get_back_vehicle() for arguments and returns.
        """
        return self.get_back_vehicle(current_scene, me, left=False, verbose=verbose)
