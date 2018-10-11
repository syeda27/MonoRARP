"""
This file defines the `risk_predictor` class.

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

from driver_risk_utils import risk_prediction_utils

class risk_predictor:
    """
    Used to keep track of some important arguments and the previous risk.
    """
    prev_risk = 0.0       # TODO make list and smooth over time

    def __init__(self,
                 H=5.0,
                 step=0.2,
                 collision_tolerance_x=2.0,
                 collision_tolerance_y=2.0,
                 ttc_tolerance=1.0):
        """
        Arguments
          H:
            Float, the simulation horizon (seconds)
          step:
            Float, the number of seconds for each step of the simulation.
          collision_tolerance_x:
            Float, tolerance (meters) to indicate a collision, laterally.
          collision_tolerance_y:
            Float, tolerance (meters) to indicate a collision, longitudinal
          ttc_tolerance:
            Float, a time-to-collision less than this value is considered a "low
              time to collision event" for the purposes of risk calculation.
        """
        self.H = H
        self.step = step
        self.collision_tolerance_x = collision_tolerance_x
        self.collision_tolerance_y = collision_tolerance_y
        self.ttc_tolerance = ttc_tolerance
        self.prev_risk = 0.0

    def reset(self):
        """
        Not a lot to reset yet, except the previous risk tracking.
        This may change in the future.
        """
        self.prev_risk = 0.0

    def get_risk(self, state, risk_type="ttc", n_sims=10, verbose=False):
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
                    self.H,
                    self.step,
                    self.collision_tolerance_x,
                    self.collision_tolerance_y,
                    verbose)
        elif risk_type.lower() == "online":
            this_scene = scene.scene(state.states,
                    ego_speed=(0.0, state.get_ego_speed()),
                    ego_accel=(0.0, 0.0))  # TODO better initialization?
            rollouts = this_scene.simulate(
                    n_sims,
                    self.H,
                    self.step,
                    verbose)
            risk = risk_prediction_utils.calculate_risk(
                    rollouts,
                    self.collision_tolerance_x,
                    self.collision_tolerance_y,
                    self.ttc_tolerance,
                    verbose)
        else:
            raise ValueError("Unsupported risk type of: {}".format(risk_type))
        self.prev_risk = (risk + self.prev_risk) / 2.0
        return self.prev_risk
