"""
This file sacrifices modularity for the purposes of increasing runtime.
Instead of simulating N rollouts and then calculating risk, which involves not
only storing all N rollouts in memory but also performing many deepcopies along
the way, this class will calculate risk while rolling out.
"""

import time
import threading
import queue

from copy import deepcopy

from risk_predictor import RiskPredictor
from scene import Scene

from driver_risk_utils import risk_prediction_utils, general_utils

class EmbeddedRiskPredictor(RiskPredictor, Scene):

    def __del__(self):
        # if a threaded copy (just overall), do not print
        if len(self.timer.start_times.keys()) == 1: return

        string = "\n=============== Ending Embedded Risk Predictor =============="
        self.timer.update_end("Overall")
        string += "\nTiming:" + self.timer.print_stats(True)
        string += "\n==============\n"
        print(string)

    def set_scene(self, state, lane_width=3.7):
        """
        Initializes the scene object with internal parameters.

        Arguments:
          state: A state object
            Used to get the vehicle_states and ego velocity (assume longitudinal)
          lane_width: float
            The standard width of a lane, in meters.
        """
        self.lane_width_m = lane_width
        self.vehicle_states = state.get_current_states()
        self.reset_scene(ego_vel=(0.0, state.get_ego_speed()),
                         ego_accel=(0.0, 0.0))
        self.means = {
                "des_v": self.ego_vel[1],    # first IDM
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

    def get_risk(self,
                 state,
                 risk_type="ttc",
                 verbose=False):
        """
        Wrapper to compute the risk for the given state.
        It also updates the internal variable: `prev_risk`.

        Arguments
          state:
            A StateHistory object to represent the road scenario and all information
            we have collected over time. TODO: this is more info than needed.
          risk_type:
            String, indicate which method to use to calculate the risk.
          verbose:
            Boolean, passed to called functions on whether to log.

        Returns
          risk:
            Float, a measure of automotive risk as calculated.

        Raises
          ValueError:
            If an unsupported risk type is used.
        """
        risk = None
        self.timer.update_start("Get Risk")

        if risk_type.lower() == "ttc":
            risk = risk_prediction_utils.calculate_ttc(
                    state,
                    self.risk_args,
                    verbose)
        elif risk_type.lower() == "online" and self.num_sims > 0:
            self.timer.update_start("Get Risk N")
            self.timer.update_start("SceneInit")
            self.set_scene(state)
            self.timer.update_end("SceneInit")
            self.timer.update_start("RiskSim")
            self.timer.update_start("CalculateRisk")
            risk = self.simulate(
                self.num_sims,
                self.sim_horizon,
                self.sim_step,
                verbose
            )
            self.timer.update_end("RiskSim", self.num_sims)
            self.timer.update_end("CalculateRisk", self.num_sims)
            self.timer.update_end("Get Risk N", self.num_sims)
        else:
            raise ValueError("Unsupported risk type of: {}".format(risk_type))
        if risk is None:
            risk = 0
        # TODO average indicated by an argument?
        self.prev_risk = (risk + self.prev_risk) / 2.0
        self.timer.update_end("Get Risk")
        if self.offline:
            # save does not average last two risk predictions
            offline_utils.save_output(risk, self.component_name, img_id, self.save_path,
                overwrite=self.overwrite_saves)
        return self.prev_risk

    def thread_deepcopy(self):
        self.timer.update_start("Thread DeepCopy")
        new_object = EmbeddedRiskPredictor(
            sim_horizon=self.sim_horizon,
            sim_step=self.sim_step,
            ttc_horizon=self.risk_args.H,
            ttc_step=self.risk_args.step,
            collision_tolerance_x=self.risk_args.collision_tolerance_x,
            collision_tolerance_y=self.risk_args.collision_tolerance_y
        )
        new_object.scene = deepcopy(self.scene)
        new_object.lane_width_m = self.lane_width_m
        new_object.vehicle_states = deepcopy(self.vehicle_states)
        new_object.ego_vel = deepcopy(self.ego_vel)
        new_object.ego_accel = deepcopy(self.ego_accel)
        new_object.means = deepcopy(self.means)
        new_object.variances = deepcopy(self.variances)
        self.timer.update_end("Thread DeepCopy")
        return new_object

    def simulate(self,
                 N=100,
                 H=5,
                 step=0.2,
                 verbose=False):
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
          threaded:
            Boolean, whether or not to thread the calculation of risk.

        Returns:
          risk: float
        """
        if verbose:
            print("Simulating {} paths for horizon {} by steps of {}".format(
                N, H, step))
        self.timer.update_start("Simulating")

        if self.max_threads > 1:
            risk = self.get_risk_threaded(N, H, step, verbose)
        else:
            risk = 0
            for i in range(N):  # TODO thread
                risk += self.simulate_one_path(H, step, verbose, self.timer)
        self.timer.update_end("Simulating", N)
        return risk / N

    def get_risk_threaded(self, N, H, step, verbose):
        risk = 0
        num_sims = 0
        self.thread_risk_queue = queue.Queue(self.max_threads)
        while num_sims < N:
            num_threads = min(self.max_threads, N-num_sims)
            list_of_threads = []
            for i in range(num_threads):
                list_of_threads.append(threading.Thread(
                    target=self.simulate_one_threaded,
                    name="path{}".format(i),
                    args=([self.thread_deepcopy(),
                           H,
                           step,
                           verbose,
                           self.timer]) # only N deepcopies
                ))
                list_of_threads[i].start()
            for t in list_of_threads:
                t.join()
            for i in range(num_threads):
                risk += self.thread_risk_queue.get()
            num_sims += num_threads
        return risk

    def simulate_one_threaded(self, scene_copy, H, step, verbose, timer=None):
        """
        scene_copy is actually a copy of this whole class...
        Timer needed as argument, not using the timer in the potential class copy
        """
        risk = scene_copy.simulate_one_path(H, step, verbose, timer)
        self.thread_risk_queue.put(risk)

    def simulate_one_path(self, H, step, verbose, timer):
        """
        Simulates one path out to time horizon H by step size (step).
        Timer needed as argument, not using the timer in the potential class copy

        Arguments:
          H: float, seconds
            The time horizon fo which to simulate to.
          step: float, seconds
            The step size to take, in seconds.
          timer: general_utils.timing object.
            The object that is keeping track of various timing qualities.

        Returns:
          risk: float
        """
        self.reset_scene(self.vehicle_states, self.ego_vel, self.ego_accel)
        for vehid in self.scene.keys():
            self.scene[vehid].longitudinal_model.randomize_parameters(
                    self.means, self.variances)
            self.scene[vehid].lateral_model.randomize_parameters(
                    self.means, self.variances)
        risk = 0
        if timer:
            timer.update_start("SimForward")
        t = 0
        while t < H:  # TODO modularize
            t += step # need while loop because range doesnt handle float step.
            risk += self.do_one_step(step, verbose, timer)

        if timer:
            timer.update_end("SimForward")
        return risk / (H * t)

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
            timer.update_start("OneSceneRisk")
        risk, _ = risk_prediction_utils.calculate_risk_one_scene(self.scene, self.risk_args, 1000, verbose)
        if timer:
            timer.update_end("OneSceneRisk", 1)
        return risk
