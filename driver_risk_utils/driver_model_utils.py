"""
Contails some utils specific to the driver models.
For example, get_accel_y() is used by the mobil_model to query the idm_model.
"""

def get_accel_y(fore_veh, back_veh, ego_vy, idm_model):
    """
    A wrapper around IDM, gets longitudinal accel for back_vehicle.
    The idm_model allows choice between using back or fore vehicle driver model.


    """
    v = back_veh.rel_vy + ego_vy
    gap = fore_veh.rel_y - back_veh.rel_y
    dV = back_veh.rel_vy - fore_veh.rel_vy # positive -> back veh approachin
    return idm_model.propagate(v, gap, dV)
