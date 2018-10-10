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
            col_tolerance_x=2.0,
            col_tolerance_y=2.0,
            verbose=True,
            col_tolerance=None):
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
        col_tolerance_x:
          Float, the collision tolerance, laterally, in meters.
          A distance less than this to another object will be considered a collision.
        col_tolerance_y:
          Float, the collision tolerance, longitudinally (forward and back), in meters.
        verbose:
          Bool, whether or not to print logging messages.
        col_tolerance:
          Float (or None), the collision tolerance for both lateral and
          longitudinal directions.

    Returns:
      Time to collision, or None if no collision within the given H.

    """
    if col_tolerance is not None:
        col_tolerance_x = col_tolerance
        col_tolerance_y = col_tolerance
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
                    abs(new_pos_x - ego_pos_x) <= col_tolerance_x) \
                    and (new_pos_y is not None and \
                    abs(new_pos_y - ego_pos_y) <= col_tolerance_y):
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
        col_tolerance_x=2,
        col_tolerance_y=2,
        verbose=True,
        col_tolerance=None):
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
          Default = 10 seconds
          Float, the horizon to compute to, in seconds.
        step:
          Default = 0.1 seconds
          Float, the granularity of the state propagation, in seconds.
        col_tolerance_x:
          Default = 2 meters
          Float, the collision tolerance, laterally, in meters.
          A distance less than this to another object will be considered a collision.
        col_tolerance_y:
          Default = 2 meters
          Float, the collision tolerance, longitudinally (forward and back), in meters.
        verbose:
          Default = True
          Bool, whether or not to print logging messages.
        col_tolerance:
          Default = None
          Float, the collision tolerance for both lateral and longitudinal directions.

    Returns:
      Time to collision, or None if no collision within the given H.
    """
    if col_tolerance is not None:
        col_tolerance_x = col_tolerance
        col_tolerance_y = col_tolerance
    t = 0
    while (t < H):
        t += step
        collision, veh_id = check_collisions(
            veh_dict,
            t,
            col_tolerance_x,
            col_tolerance_y)
        if collision:
            if verbose:
                print("In calculate_ttc:")
                print("veh id {} colliding in {} seconds".format(veh_id))
            return t
    return None

def check_collisions(veh_dict, t, col_tol_x, col_tol_y):
    """
    Simply check the vehicle positions after t time assuming constant
    velocity and given the initial veh_dict for any collisions
        (respecting the given collision tolerances).
    """
    ego_pos_x = 0 # for now, assume no lateral motion.
    ego_pos_y = veh_dict["ego"].rel_vy * t
    for veh_id in veh_dict.keys():
        if veh_id == "ego": continue
        new_pos_x = veh_dict[veh_id].rel_x + veh_dict[veh_id].rel_vx*t
        new_pos_y = veh_dict[veh_id].rel_y + veh_dict[veh_id].rel_vy*t
        if abs(new_pos_x - ego_pos_x) <= col_tol_x \
                and abs(new_pos_y - ego_pos_y) <= col_tol_y:
            return True, veh_id
    return False, "No collisions detected."

def calculate_risk(rollouts, tol_x, tol_y, tol_ttc, verbose=False):
    """
    Calculates automotive risk from a series of rollouts.
    """
    risk = 0.0
    lowest_ttc = 10000
    for path in rollouts:
        rollout_risk = 0.0
        for curr_scene in path:
            colliding, vehid = check_collisions(curr_scene, 0, tol_x, tol_y)
            if colliding:
                rollout_risk += 10
            else:
                ttc = calculate_ttc_veh(curr_scene, 2, 0.2, tol_x, tol_y, verbose)
                if ttc:
                    if ttc < lowest_ttc:
                        lowest_ttc = ttc
                    if ttc < tol_ttc:
                        rollout_risk += 1
        risk += rollout_risk / len(path)
    risk = risk / len(rollouts)
    if verbose:
        if risk > 0:
            print("lowest ttc:", lowest_ttc)
            print("risk:", risk)
    return risk
