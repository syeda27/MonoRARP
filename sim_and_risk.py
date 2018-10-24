"""
This file sacrifices modularity for the purposes of increasing runtime.
Instead of simulating N rollouts and then calculating risk, which involves not
only storing all N rollouts in memory but also performing many deepcopies along
the way, this class will calculate risk while rolling out, and only have to do
N deepcopies (for the initialization).
"""

import risk_predictor, scene

class embedded_risk_predictor(risk_predictor):
    def get_risk(self,
                 state,
                 risk_type="ttc",
                 n_sims=10,
                 verbose=False,
                 profile=False):
        """
        Wrapper to compute the risk for the given state.
        It also updates the internal variable: `prev_risk`.

        Arguments
          state:
            A state object to represent the current road scenario--obj_det_state
          n_sims:
            Integer, if using the `online` method, determins how many sets of
              rollouts to simulate.
          verbose:
            Boolean, passed to called functions on whether to log.
          profile:
            Boolean, whether or not to print timings of functions.

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
            if profile:
                start = time.time()
            this_scene = sim_and_risk.sim_and_risk(
                    state.get_current_states(),
                    ego_vel=(0.0, state.get_ego_speed()),
                    ego_accel=(0.0, 0.0))  # TODO better initialization?
            if profile:
                print("SceneInit took: {}".format(time.time() - start))
                start = time.time()
            risk = this_scene.simulate(
                n_sims,
                self.sim_horizon,
                self.sim_step,
                verbose
            )
            if profile:
                print("RiskSim took: {}".format(time.time() - start))
                start = time.time()
            if profile:
                print("CalculateRisk took: {}".format(time.time() - start))
        else:
            raise ValueError("Unsupported risk type of: {}".format(risk_type))
        self.prev_risk = (risk + self.prev_risk) / 2.0
        return self.prev_risk

class sim_and_risk(scene):
    def simulate(self, N=100, H=5, step=0.2, verbose=False, profile=False):
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
          profile: boolean
            Whether or not to print timings of functions.

        Returns:
          risk: float
        """
        if verbose:
            print("Simulating {} paths for horizon {} by steps of {}".format(
                N, H, step))
        timer = None # defines the variable in case not profiling.
        risk = 0 # list of N paths, which are snapshots of the scenes
        if profile:
            timer = general_utils.timing(
                ["Simulating", "Deepcopies", "SimForward", "GetAction", "SceneUpdate"])
            timer.update_start("Simulating")

        for i in range(N):  # TODO modularize
            risk += self.simulate_one_path(H, step, verbose, profile, timer)
        if profile:
            timer.update_end("Simulating", N)
            timer.print_stats()
        return risk / N

    def simulate_one_path(self, H, step, verbose, profile, timer):
        """
        Simulates one path out to time horizon H by step size (step).

        Arguments:
          H: float, seconds
            The time horizon fo which to simulate to.
          step: float, seconds
            The step size to take, in seconds.
          profile: boolean
            Whether or not to print timings of functions.
          timer: general_utils.timing object. None if profile == False
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
        if profile:
            timer.update_start("SimForward")
        t = 0
        while t < H:  # TODO modularize
            t += step # need while loop because range doesnt handle float step.
            risk += self.do_one_step(step, verbose, profile, timer)

        if profile:
            timer.update_end("SimForward")
        return risk / (H / t)

    def do_one_step(self, step, verbose, profile, timer):
        """
        Does one step of path generation.
        This involves computing the actions for each vehicle in the scene,
          and updating the scene with the appropriate actions and step size.

        Arguments:
          step: float, seconds
            The step size to take, in seconds.
          verbose: boolean
            Whether or not to log various occurrences.
          profile: boolean
            Whether or not to print timings of functions.
          timer: general_utils.timing object. None if profile == False
            The object that is keeping track of various timing qualities.

        Returns:
          scene_here: dict< string : dictionary <string:float> >
            VehicleID : state information like relative position, velocity, etc.
            This variable is what is simulated, and we must create a copy
              through the use of deepcopy() to return.

        """
        if profile:
            timer.update_start("GetAction")

        actions = {}
        for vehid in self.scene.keys():
            actions[vehid] = self.scene[vehid].get_action(self, step) # dvxdt, dvydt
            if verbose:
                print("action for", vehid, ":", actions[vehid])
        if profile:
            timer.update_end("GetAction", 1)
            timer.update_start("SceneUpdate")

        self.update_scene(actions, step)
        risk = risk_prediction_utils.calculate_risk_one_scene(self.scene, self.risk_args, 1000, verbose)
        return risk
