"""
This file defines the `RiskPredictor` class.

The main class function works in conjunction with the StateHistory

It uses the information from the state to calculate the automotive risk in
the future.

It will be built up over time to include a variety of methods.
Initially, it will use relative position and velocity to simply propagate
  every car forward in time and determine if there are any collisions.
  If there are, it will return the time to the collision (TTC)

What does happen:
Risk predictor either
 (1) calculates ttc from current state (type == ttc)
 (2) creates a scene, calls the simulator for monte carlo rollouts, and
     calculates risk (see utils) based on those rollouts.

What should (maybe) happen:
(A)
Risk predictor simulates to a certain point (creating rollouts).
Risk predictor calls risk estimator on each of those given rollouts.
Risk estimator evaluates the scene for a "risk" score, at each t along the
  simulated trajectory (the rollout).

(B)
Alternatively, offline risk predictor just given an initial scene and
predict risk into the future.
Could train based on current outputs.

(C)
Alternatively, risk predictor simulates to a certain point.
Risk predictor calls an offline risk estimator that does not calculate ttc
or anything like that?

"""

import numpy as np
import sys
sys.stdout.flush()
import scene
import time

from driver_risk_utils import risk_prediction_utils, general_utils

class RiskPredictor:
    """
    Used to keep track of some important arguments and the previous risk.
    """
    prev_risk = 0.0       # TODO make list and smooth over time

    def __init__(self,
                 num_sims=1,
                 sim_horizon=5.0,
                 sim_step=0.2,
                 ttc_horizon=3.0,
                 ttc_step=0.25,
                 collision_tolerance_x=2.0,
                 collision_tolerance_y=2.0,
                 max_threads=10):
        """
        Arguments
          num_sims:
            Integer, the number of simulations to run when getting risk.
          sim_horizon:
            Float, the simulation horizon (seconds).
          sim_step:
            Float, the number of seconds for each step of the simulation.
          ttc_horizon:
            Float, the horizon (seconds) to use when finding low ttc events.
          ttc_step:
            Float, the step size (seconds) to use when finding low ttc events.
          collision_tolerance_x:
            Float, tolerance (meters) to indicate a collision, laterally.
          collision_tolerance_y:
            Float, tolerance (meters) to indicate a collision, longitudinal
          max_threads:
            Int, the maximum number of threads to spawn at any given time.
            Setting this number to <= 1 will force the non-threaded method.
        """
        self.num_sims = num_sims
        self.sim_horizon = sim_horizon
        self.sim_step = sim_step
        self.risk_args = risk_prediction_utils.RiskArgs(
            ttc_horizon,
            ttc_step,
            collision_tolerance_x,
            collision_tolerance_y,
            collision_score=10,
            low_ttc_score=1
        )
        self.prev_risk = 0.0
        self.max_threads = max_threads
        self.timer = general_utils.Timing()
        self.timer.update_start("Overall")

    def __del__(self):
        string = "\n=============== Ending Risk Predictor =============="
        self.timer.update_end("Overall")
        string += "\nTiming:" + self.timer.print_stats(True)
        string += "\n==============\n"
        print(string)


    def reset(self):
        """
        Not a lot to reset yet, except the previous risk tracking.
        This may change in the future.
        """
        self.prev_risk = 0.0

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
            this_scene = scene.Scene(
                    state.get_current_states(),
                    ego_vel=(0.0, state.get_ego_speed()),
                    ego_accel=(0.0, 0.0))  # TODO better initialization?
            self.timer.update_start("RiskSim")
            # TODO use self.max_threads for both making rollouts and calculating risk.
            rollouts = this_scene.simulate(
                    self.num_sims,
                    self.sim_horizon,
                    self.sim_step,
                    verbose,
                    self.timer)
            self.timer.update_end("RiskSim", self.num_sims)
            self.timer.update_start("CalculateRisk")
            risk = risk_prediction_utils.calculate_risk(
                    rollouts,
                    self.risk_args,
                    verbose)
            self.timer.update_end("CalculateRisk", self.num_sims)
            self.timer.update_end("Get Risk N", self.num_sims)
        else:
            raise ValueError("Unsupported risk type of: {}".format(risk_type))
        if risk is None:
            risk = 0
        self.prev_risk = (risk + self.prev_risk) / 2.0
        self.timer.update_end("Get Risk")
        return self.prev_risk
