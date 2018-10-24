"""
This file contains some utilities related to risk prediction that do not
rely on any specifics from the risk_predictor. If the function does rely
on that information, it must be passed in. This allows the functions to be
used when no risk_predictor object is defined. Thus, they are utilities.
"""

class risk_args:
    """
    risk_args is a class to help track some important arguments for the risk
    predictor utilties.
    """
    def __init__(self,
                 H=10.0,
                 step=0.1,
                 collision_tolerance_x=2.0,
                 collision_tolerance_y=2.0,
                 collision_score=10.0,
                 low_ttc_score=1.0):
        """
        H:
          Float, the horizon to compute to, in seconds. This will be used for
          finding low ttc events. We check the time up to H, and if no
          collisions, we know there are no low ttc events.
        step:
          Float, the granularity of the state propagation, in seconds.
        collision_tolerance_x:
          Float, the collision tolerance, laterally, in meters.
          A distance less than this to another object will be considered a collision.
        collision_tolerance_y:
          Float, the collision tolerance, longitudinally (forward and back), in meters.
        collision_score:
          Float. The score assigned to a collision event when calculating risk.
        low_ttc_score:
          Float. The score assigned to a low time to collision event when
          calculating risk.
        """
        self.H = H
        self.step = step
        self.collision_tolerance_x = collision_tolerance_x
        self.collision_tolerance_y = collision_tolerance_y
        self.collision_score = collision_score
        self.low_ttc_score = low_ttc_score

def calculate_ttc(
            state,
            risk_args,
            verbose=True):
    """
    This first method to calculate ttc (time-to-collision) is just brute force.

    Propagate the scene forward by tenths of a second until a collision
    is detected, or the maximum horizon is reached.

    Assume horizontal speed for the ego car is 0.
    Assume all accelerations are 0.

    Arguments:
        state:
          the State object that is used to get current positions / speeds for vehicles.
        risk_args:
          the parameters class for risk predictor, containing parameters like
          collisions tolerances, horizon to use, etc.
        verbose:
          Bool, whether or not to print logging messages.
        collision_tolerance:
          Float (or None), the collision tolerance for both lateral and
          longitudinal directions.

    Returns:
      Time to collision, or None if no collision within the given H.

    """
    t = 0
    while (t < risk_args.H):
        t += risk_args.step
        ego_pos_x = 0 # for now, assume no lateral motion.
        ego_pos_y = state.get_ego_speed() * t
        for veh_id in state.states.keys():
            this_state = state.states[veh_id][-1]
            new_pos_x = None
            new_pos_y = None
            if "distance_x" in this_state and "speed_x" in this_state:
                new_pos_x = this_state["distance_x"] + this_state["speed_x"]*t
            if "distance_y" in this_state and "speed_y" in this_state:
                new_pos_y = this_state["distance_y"] + this_state["speed_y"]*t

            if (new_pos_x is not None and \
                    abs(new_pos_x - ego_pos_x) <= risk_args.collision_tolerance_x) \
                    and (new_pos_y is not None and \
                    abs(new_pos_y - ego_pos_y) <= risk_args.collision_tolerance_y):
                if verbose:
                    print("calculate_ttc")
                    print(new_pos_x, ego_pos_x)
                    print(new_pos_y, ego_pos_y)
                    print("veh id", veh_id, "colliding in", t, "seconds")
                return t
    return None


def calculate_ttc_veh(veh_dict, risk_args, verbose=True):
    """
    This second method to calculate ttc (time-to-collision) is just brute force.
    It is the same as calculate_ttc, but with vehicles instead of states.

    Propagate the scene forward by tenths of a second until a collision
    is detected, or the maximum horizon is reached.

    Assume horizontal speed for the ego car is 0.
    Assume all accelerations are 0.

    Arguments:
        veh_dict:
          A dictionary of all of the vehicles in the scene, with their information.
        risk_args:
          the parameters class for risk predictor, containing parameters like
          collisions tolerances, horizon to use, etc.
        verbose:
          Bool, whether or not to print logging messages.

    Returns:
      Time to collision, or None if no collision within the given H.
    """
    t = 0
    while (t < risk_args.H):
        t += risk_args.step
        collision, veh_id = check_collisions(
            veh_dict,
            t,
            risk_args.collision_tolerance_x,
            risk_args.collision_tolerance_y)
        if collision:
            if verbose:
                print("In calculate_ttc:")
                print("veh id {} colliding in {} seconds".format(veh_id))
            return t
    return None

def check_collisions(veh_dict,
                     t,
                     collision_tolerance_x,
                     collision_tolerance_y):
    """
    Simply check the vehicle positions after t time assuming constant
    velocity and given the initial veh_dict for any collisions
        (respecting the given collision tolerances).

    Arguments
      veh_dict:
        A dictionary of all of the vehicles in the scene, with their information.
      t:
        Float, the time at which we want to check for collisions, forward in
        time from where the veh_dict says every car is, in seconds.
      collision_tolerance_x:
        Float, the collision tolerance, laterally, in meters.
        A distance less than this to another object will be considered a collision.
      collision_tolerance_y:
        Float, the collision tolerance, longitudinally (forward and back), in meters.

    Returns
      (collision, msg):
         collision: Boolean, whether or not a collision occurred.
         msg: String, the vehicle id or a message that no collision occurred.
    """
    ego_pos_x = 0 # for now, assume no lateral motion.
    ego_pos_y = veh_dict["ego"].rel_vy * t
    for veh_id in veh_dict.keys():
        if veh_id == "ego": continue
        new_pos_x = veh_dict[veh_id].rel_x + veh_dict[veh_id].rel_vx*t
        new_pos_y = veh_dict[veh_id].rel_y + veh_dict[veh_id].rel_vy*t
        if abs(new_pos_x - ego_pos_x) <= collision_tolerance_x \
                and abs(new_pos_y - ego_pos_y) <= collision_tolerance_y:
            return True, veh_id
    return False, "No collisions detected."

def calculate_risk(rollouts,
                   risk_args,
                   verbose=False):
    """
    Calculates automotive risk from a series of rollouts.

    Arguments
        rollouts:
          A list of paths that were simulated.
          Each path is a list of scenes.
          Each scene is the vehicle dictionary we see in previous functions.
        risk_args:
          the parameters class for risk predictor, containing parameters like
          collisions tolerances, horizon to use, etc.
        verbose:
          Bool, whether or not to print logging messages.

    Returns
      Risk: float score for the estimated automotive risk in the given rollouts.
    """
    risk = 0.0
    lowest_ttc = 10000
    for path in rollouts:
        risk_from_path, lowest_ttc = calculate_risk_one_rollout(
            path, risk_args, lowest_ttc, verbose)
        risk += risk_from_path
    risk = risk / len(rollouts)
    if verbose:
        if risk > 0:
            print("lowest ttc:", lowest_ttc)
            print("risk:", risk)
    return risk

def calculate_risk_one_rollout(path, risk_args, lowest_ttc=1000, verbose=False):
    """
    Calculates automotive risk for a single rollout. This is the estimated
    risk for the given path.
    We define the risk, for now, to be:
      (A) risk_args.collision_score for each scene with a collision.
      (B) risk_args.low_ttc_score for each scene with at least one low time
          to collision event including the ego vehicle.
    This risk is averaged over the number of scenes in the path.

    Arguments
        path:
          A list of scenes.
          Each scene is the vehicle dictionary we see in previous functions.
        risk_args:
          the parameters class for risk predictor, containing parameters like
          collisions tolerances, horizon to use, etc.
        lowest_ttc:
          Used exclusively for logging purposes. Keeps track of what the lowest
          time to collision we have encountered is.
        verbose:
          Bool, whether or not to print logging messages.

    Returns
      Risk: float score for the estimated automotive risk in the path.
    """
    rollout_risk = 0.0
    for curr_scene in path:
        colliding, vehid = check_collisions(
            curr_scene,
            0.0,
            risk_args.collision_tolerance_x,
            risk_args.collision_tolerance_y)
        if colliding:
            rollout_risk += risk_args.collision_score
        else:
            # we simulate forward in time using constant velocities.
            ttc = calculate_ttc_veh(curr_scene,
                                    risk_args,
                                    verbose)
            if ttc: # means we encountered a collision in less than risk_args.H
                lowest_ttc = min(ttc, lowest_ttc)
                rollout_risk += risk_args.low_ttc_score
    return rollout_risk / len(path), lowest_ttc
