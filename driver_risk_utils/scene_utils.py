import vehicle


def get_fore_vehicle(lane_width, current_scene, me, verbose=False):
    """
    Gets the vehicle in front of "me".
    "me" is a vehicle object that is not necessarily the ego vehicle, just the
    vehicle we are getting the neighbor with respect to.

    Arguments:
      lane_width: float, meters.
        The width of the lane in this scene, in meters.
      current_scene: dict< string : dictionary <string:float> >
        VehicleID : state information like relative position, velocity, etc.
      me: Vehicle
        A Vehicle object that exists in the current scene.
      verbose: boolean
        Whether or not to log various occurrences.

    Returns:
      A Vehicle object
    """
    best = None
    closest_y = 1000
    for vehid in current_scene.keys():
        if vehid == me.veh_id: continue
        them = current_scene[vehid]
        if abs(them.rel_x - me.rel_x) <= (lane_width / 2.0):
            gap = them.rel_y - me.rel_y
            if gap < closest_y and gap > 0:
                closest_y = gap
                best = them
    if best is None:
        best = vehicle.Vehicle("fake_veh",
            {"speed_x": me.rel_vx,
             "speed_y": me.rel_vy,          # same speed
             "distance_y": me.rel_y + 1000, # largest gap
             "distance_x": me.rel_x + 10})  # unused
    if verbose:
        print("me: ", me.veh_id, "fore: ", best.veh_id)
    return best

def get_back_vehicle(lane_width, current_scene, me, left=True, verbose=False):
    """
    Gets the vehicle behind "me", in either the lane to the left, or the lane
    to the right.
    See driver_models.py for the motivation for this function, specifically the
    MOBIL model.

    "me" is a vehicle object that is not necessarily the ego vehicle, just
    the vehicle we are getting the neighbor with respect to.

    # vehicle.rel_x is negative if left

    Arguments:
      lane_width: float, meters.
        The width of the lane in this scene, in meters.
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
      A Vehicle object. A "Fake vehicle" that is guaranteed not to interact with
        "me" if there is no vehicle that satisfies the criteria.
    """
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
        if dx > lane_width * 0.5 and dx < lane_width * 1.5:
            gap = me.rel_y - them.rel_y # positive when me is in front
            if gap < closest_y and gap > 0:
                closest_y = gap
                best = them
    if best is None:
        lane_x = lane_width
        if left:
            lane_x = -lane_x
        best = vehicle.Vehicle("fake_veh",
            {"speed_x": me.rel_vx,
             "speed_y": me.rel_vy,              # same speed
             "distance_y": me.rel_y - 1000,     # largest gap
             "distance_x": me.rel_x - lane_x})  # in other lane
    return best
