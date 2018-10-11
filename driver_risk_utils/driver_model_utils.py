"""
This file contails some utility functions specific to the driver models.
For example, get_accel_y() is used by the mobil_model to query the idm_model.
"""

def get_accel_y(fore_veh, back_veh, ego_vy, idm_model):
    """
    A wrapper around IDM, given a front and a back vehicle (and the absolute
    speed), and a longitudinal driver model, this function returns the
    desired acceleration of the back vehicle.

    Arguments
      fore_vehicle:
        A vehicle object, for the vehicle that is currently in front.
      back_vehicle:
        A vehicle object, representing the vehicle that is behind the fore_veh.
      ego_vy:
        A float, indicating the absolute speed of the ego vehicle. This is
        necessary because the vehicle objects only contain relative speeds.
      idm_model:
        A longitudinal driver model consisting of a propagate() function.
    """
    v = back_veh.rel_vy + ego_vy
    gap = fore_veh.rel_y - back_veh.rel_y
    dV = back_veh.rel_vy - fore_veh.rel_vy # positive -> back veh approachin
    return idm_model.propagate(v, gap, dV)
