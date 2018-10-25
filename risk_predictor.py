"""
This file defines the `RiskPredictor` class.

The main class function works in conjunction with the STATE from obj_det_state

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

from driver_risk_utils import risk_prediction_utils

class RiskPredictor:
    """
    Used to keep track of some important arguments and the previous risk.
    """
    prev_risk = 0.0       # TODO make list and smooth over time

    def __init__(self,
                 sim_horizon=5.0,
                 sim_step=0.2,
                 ttc_horizon=3.0,
                 ttc_step=0.25,
                 collision_tolerance_x=2.0,
                 collision_tolerance_y=2.0):
        """
        Arguments
          sim_horizon:
            Float, the simulation horizon (seconds)
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
        """
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

    def reset(self):
        """
        Not a lot to reset yet, except the previous risk tracking.
        This may change in the future.
        """
        self.prev_risk = 0.0

    def get_risk(self,
                 state,
                 risk_type="ttc",
                 n_sims=10,
                 verbose=False,
                 timer=None):
        """
        Wrapper to compute the risk for the given state.
        It also updates the internal variable: `prev_risk`.

        Arguments
          state:
            A state object to represent the current road scenario--obj_det_state
          risk_type:
            String, indicate which method to use to calculate the risk.
          n_sims:
            Integer, if using the `online` method, determins how many sets of
              rollouts to simulate.
          verbose:
            Boolean, passed to called functions on whether to log.
          timer: general_utils.timing object.
            The object that is keeping track of various timing qualities.

        Returns
          risk:
            Float, a measure of automotive risk as calculated.

        Raises
          ValueError:
            If an unsupported risk type is used.
        """
        if risk_type.lower() == "ttc":
            risk = risk_prediction_utils.calculate_ttc(
                    state,
                    self.risk_args,
                    verbose)
        elif risk_type.lower() == "online":
            if timer:
                timer.update_start("SceneInit")
            this_scene = scene.Scene(
                    state.get_current_states(),
                    ego_vel=(0.0, state.get_ego_speed()),
                    ego_accel=(0.0, 0.0))  # TODO better initialization?
            if timer:
                timer.update_end("SceneInit")
                timer.update_start("RiskSim")
            rollouts = this_scene.simulate(
                    n_sims,
                    self.sim_horizon,
                    self.sim_step,
                    verbose,
                    timer)
            if timer:
                timer.update_end("RiskSim", n_sims)
                timer.update_start("CalculateRisk")
            risk = risk_prediction_utils.calculate_risk(
                    rollouts,
                    self.risk_args,
                    verbose)
            if timer:
                timer.update_end("CalculateRisk")
        else:
            raise ValueError("Unsupported risk type of: {}".format(risk_type))
        self.prev_risk = (risk + self.prev_risk) / 2.0
        return self.prev_risk
