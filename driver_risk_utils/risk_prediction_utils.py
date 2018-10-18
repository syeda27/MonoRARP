"""
This file contains some utilities related to risk prediction that do not
rely on any specifics from the risk_predictor. If the function does rely
on that information, it must be passed in. This allows the functions to be
used when no risk_predictor object is defined. Thus, they are utilities.
"""

def calculate_ttc(
            state,
            H=10.0,
            step=0.1,
            collision_tolerance_x=2.0,
            collision_tolerance_y=2.0,
            verbose=True,
            collision_tolerance=None):
    """
    This first method to calculate ttc (time-to-collision) is just brute force.

    Propagate the scene forward by tenths of a second until a collision
    is detected, or the maximum horizon is reached.

    Assume horizontal speed for the ego car is 0.
    Assume all accelerations are 0.

    Arguments:
        state:
          the State object that is used to get current positions / speeds for vehicles.
        H:
          Float, the horizon to compute to, in seconds.
        step:
          Float, the granularity of the state propagation, in seconds.
        collision_tolerance_x:
          Float, the collision tolerance, laterally, in meters.
          A distance less than this to another object will be considered a collision.
        collision_tolerance_y:
          Float, the collision tolerance, longitudinally (forward and back), in meters.
        verbose:
          Bool, whether or not to print logging messages.
        collision_tolerance:
          Float (or None), the collision tolerance for both lateral and
          longitudinal directions.

    Returns:
      Time to collision, or None if no collision within the given H.

    """
    if collision_tolerance is not None:
        collision_tolerance_x = collision_tolerance
        collision_tolerance_y = collision_tolerance
    t = 0
    while (t < H):
        t += step
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
                    abs(new_pos_x - ego_pos_x) <= collision_tolerance_x) \
                    and (new_pos_y is not None and \
                    abs(new_pos_y - ego_pos_y) <= collision_tolerance_y):
                if verbose:
                    print("calculate_ttc")
                    print(new_pos_x, ego_pos_x)
                    print(new_pos_y, ego_pos_y)
                    print("veh id", veh_id, "colliding in", t, "seconds")
                return t
    return None


def calculate_ttc_veh(veh_dict,
        H=10,
        step=0.1,
        collision_tolerance_x=2,
        collision_tolerance_y=2,
        verbose=True,
        collision_tolerance=None):
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
        H:
          Float, the horizon to compute to, in seconds.
        step:
          Float, the granularity of the state propagation, in seconds.
        collision_tolerance_x:
          Float, the collision tolerance, laterally, in meters.
          A distance less than this to another object will be considered a collision.
        collision_tolerance_y:
          Float, the collision tolerance, longitudinally (forward and back), in meters.
        verbose:
          Bool, whether or not to print logging messages.
        collision_tolerance:
          Float (or None), the collision tolerance for both lateral and
          longitudinal directions.

    Returns:
      Time to collision, or None if no collision within the given H.
    """
    if collision_tolerance is not None:
        collision_tolerance_x = collision_tolerance
        collision_tolerance_y = collision_tolerance
    t = 0
    while (t < H):
        t += step
        collision, veh_id = check_collisions(
            veh_dict,
            t,
            collision_tolerance_x,
            collision_tolerance_y)
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
                   collision_tolerance_x,
                   collision_tolerance_y,
                   tolerance_ttc,
                   verbose=False):
    """
    Calculates automotive risk from a series of rollouts.

    Arguments
        rollouts:
          A list of paths that were simulated.
          Each path is a list of scenes.
          Each scene is the vehicle dictionary we see in previous functions.
        collision_tolerance_x:
          Float, the collision tolerance, laterally, in meters.
          A distance less than this to another object will be considered a collision.
        collision_tolerance_y:
          Float, the collision tolerance, longitudinally (forward and back), in meters.
        tolerance_ttc:
          Float, a time-to-collision less than this value is considered a "low
            time to collision event" for the purposes of risk calculation.
        verbose:
          Bool, whether or not to print logging messages.

    Returns
      Risk: float score for the estimated automotive risk in the give rollouts.
    """
    risk = 0.0
    lowest_ttc = 10000
    for path in rollouts:
        rollout_risk = 0.0
        for curr_scene in path:
            colliding, vehid = check_collisions(
                curr_scene,
                0.0,
                collision_tolerance_x,
                collision_tolerance_y)
            if colliding:
                rollout_risk += 10.0        # TODO args
            else:
                ttc = calculate_ttc_veh(
                    curr_scene,
                    2.0,                    # TODO args
                    0.2,                    # TODO args
                    collision_tolerance_x,
                    collision_tolerance_y,
                    verbose)
                if ttc:
                    if ttc < lowest_ttc:
                        lowest_ttc = ttc
                    if ttc < tolerance_ttc:
                        rollout_risk += 1.0 # TODO args
        risk += rollout_risk / len(path)
    risk = risk / len(rollouts)
    if verbose:
        if risk > 0:
            print("lowest ttc:", lowest_ttc)
            print("risk:", risk)
    return risk