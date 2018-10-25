"""
This file contains classes for the driver models used. In particular, for now
it containes the idm_model for longitudinal acceleration, and the mobil_model
for lateral acceleration.

Sources:
  http://traffic-simulation.de/IDM.html
  http://traffic-simulation.de/MOBIL.html
  And wikipedia, of course.
"""
import numpy as np
from driver_risk_utils import driver_model_utils

# TODO make an abstract class and have these extend them?

class IDM_Model:
    """
    The intelligent driver model (IDM) is used to model the longitudinal
    acceleration for human drivers.
    """
    def __init__(self, des_v=30, hdwy_t=1.5, min_gap=2.0, accel=0.3, deccel=3.0):
        """
        Arguments
          des_v:
            Float, the desired velocity of this driver, if there was an
            open road (m/s).
          hdwy_t:
            The desired time-headway of this driver (seconds).
            This means the driver would like be able to cover the distance to
            the leading vehicle in at least this amount of time, at constant
            velocity.
          min_gap:
            The minimum distance between the driver and leading vehicle (m).
            If the gap is smaller than this, the driver will engage in emergency
            braking.
          accel:
            The maximum acceleration for this driver (m/s^2).
          deccel:
            The maximum braking deceleration, a positive number (m/s^2)
        """
        self.v0 = des_v
        self.T = hdwy_t
        self.s0 = min_gap
        self.a = accel
        self.b = deccel
        self.delta = 4 # Acceleration exponent, according to wikipedia

    def randomize_parameters(self, means, variances, debug=False):
        """
        Call to randomize the internal parameters of the driver model.
        Omiting a parameter name in either the mean or variance dictionary will
        simply result in the parameter not being randomized.
        Extraneous keys will be ignored, but will be printed if in debug mode.


        Arguments
          means: dictionary of `parameter_name` : `mean` for all parameters of
            the driver model that we would like to randomize. Omitting a
            parameter means we will not randomize it.
          variances: dictionary of `parameter_name` : `variance` for all parameters of
            the driver model that we would like to randomize. Omitting a
            parameter means we will not randomize it. As a reminder, standard
            deviation is the square root of the variance.
          debug: boolean for whether or not to print unusual situations.
        """
        for key in means.keys():
            if key not in variances:
                if debug:
                    print("Key {} appears in means for IDM,"
                          " but not variances.".format(key))
                continue
            elif key is "des_v":
                self.v0 = np.random.normal(means[key], np.sqrt(variances[key]))
            elif key is "hdwy_t":
                self.T =  np.random.normal(means[key], np.sqrt(variances[key]))
            elif key is "min_gap":
                self.s0 = np.random.normal(means[key], np.sqrt(variances[key]))
            elif key is "accel":
                self.a = np.random.normal(means[key], np.sqrt(variances[key]))
            elif key is "deccel":
                self.b = np.random.normal(means[key], np.sqrt(variances[key]))
            elif debug:
                print("Invalid parameter to randomize (IDM): {}".format(key))

    def propagate(self, v, s, dV, verbose=False):
        """
        Calculate the acceleration given current state info.
        If the car is really close to the front car, engage in emergency
        breaking by returning the maximum deceleration.

        Arguments
          v:
            Float, the speed of the ego vehicle (m/s).
          s:
            Float, bumper - to - bumper - gap between the ego vehicle and
            the leading vehicle (m).
          dV:
            Float, relative speed between ego vehicle and fore vehicle (m/s).
            This value is positive when the ego vehicle is faster than the
            leading vehicle.
          verbose:
            Boolean, whether or not to print extra information.

        Returns
          dv_dt:
            The acceleration (an action) the driver should take according to
            this model. (m/s^2)
        """
        if verbose:
            print(v, s, dV)
            print(self.s0, self.v0)
        if s <= self.s0: # min gap
            if verbose:
                print("The gap is:", s)
                if v < 0.0:
                    print("Vehicle is travelling in reverse!")
                    return 0.0
            return -self.b # emergency braking.

        denominator = 2.0 * np.sqrt(np.abs(self.a * self.b))
        s_star = self.s0 + max(0.0, v * self.T + (v * dV) / denominator)

        dv_dt_free_road = self.a * (1.0 - np.power(v / self.v0, self.delta))
        dv_dt_interaction = -self.a * np.power(s_star / s, 2.0)
        dv_dt = dv_dt_free_road + dv_dt_interaction

        return dv_dt


# MOBIL for latitudinal acceleration
class MOBIL_Model:
    """
    The mobil model is used to model the lateral acceleration for human drivers.

    Terminology:
              <Target Lane>
    |         |         |
    |   ____  |         |
    |  |    | |         |
    |  | B  | |         |
    |  |____| |         |      /|\
    |         |   ____  |       |
    |         |  |    | |       |
    |         |  | D  | |       | Traffic flows from bottom to top
    |   ____  |  |____| |       | in this example.
    |  |    | |         |
    |  | A* | |         |
    |  |____| |         |
    |         |   ____  |
    |         |  |    | |
    |         |  | C  | |
    |         |  |____| |

    Target Lane:
      The lane we want to evaluate if we should move into.
      Not determined within the model. We just get the vehicles.
    A*:
      `ego_vehicle`. The driver / car we want to find the acceleration for.
    B:
      `fore_vehicle`. The vehicle in front of the ego car, in our lane.
      If this vehicle is going too slowly, we are incentivized to change lanes.
    C:
      `back_vehicle`. The vehicle behind the ego car, in the target lane.
      This is the vehicle that would have to slow down if A changed lanes.
    D:
      `back_vehicles_fore_vehicle`. The vehicle in front of C. This vehicle is
      in the the target lane. It is not necessarily guaranteed to be in front of
      the ego vehicle (it could be on level with the ego vehicle).
      We need this vehicle to know the current and projected gap that we would
      want to move into if we changed lanes.

    """
    def __init__(self, p=0.2, b_safe=3, a_thr=0.2, delta_b=0):
        """
        Arguments
          p:
            Float, the `politeness` factor for the driver. A more polite
            driver is less likely to make aggressive lane changes.
          b_safe:
            Float, the safe braking acceleration (positive, m/s).
          a_thr:
            Float, acceleration threshold, below IDM.a. This is used to
            calculate the `incentive_criterion` for changing lanes.
          delta_b:
            Float, the bias towards right lane.

        """
        self.p = p
        self.b_safe = b_safe
        self.a_thr = a_thr
        self.delta_b = delta_b

    """
    this_vehicle is the vehicle we are making a decision for
    fore_vehicle is the vehicle that is currently in front of us.
    back_vehicle is the vehicle that is behind us in the target lane,
        the one that would
        likely have to decelerate in the event of us moving over
    backs_fore_veh is the vehicle that is currently in front of
        the back_vehicle
    ego_vy is the absolute longitudinal speed that everyting is relative to.

    Returns a lateral acceleration that would have this_vehicle move to
        the lateral position of back_vehicle in 1 time step
    """
    def propagate(self,
                  this_vehicle,
                  fore_vehicle,
                  back_vehicle,
                  backs_fore_veh,
                  ego_vy=15,
                  step=0.2):
        will_change_lanes = self.safety_criterion(
            this_vehicle,
            back_vehicle,
            ego_vy
        ) and self.incentive_criterion(
            this_vehicle,
            back_vehicle,
            fore_vehicle,
            backs_fore_veh,
            ego_vy
        )
        if not will_change_lanes:
            return 0
        return (back_vehicle.rel_x - this_vehicle.rel_x) / step

    def safety_criterion(self, this_vehicle, back_vehicle, ego_vy=15.0):
        """
        Returns a boolean for whether this vehicle moving into the target lane
            would satisfy the safety criterion, as determined by our IDM model.
            In other words, making sure we would not cause the other vehicle to
            perform an emergency braking maneuver.
        We could use the back vehicle's IDM instead, but that is unknown to the
            driver and I think it is more realistic to use our own model

        Arguments
          this_vehicle:
            A vehicle object, representing the vehicle that we want to check
            if it should or should not change lanes.
          back_vehicle:
            A vehicle object, representing the vehicle that would be behind
            the ego vehicle if it changed lanes.
          ego_vy:
            A float, indicating the absolute speed of the ego vehicle. This is
            necessary because the vehicle objects only contain relative speeds.

        Returns
          Boolean: True if changing lanes would NOT cause the back_vehicle to
            engage in emergency braking.
        """
        v = back_vehicle.rel_vy + ego_vy
        new_gap = this_vehicle.rel_y - back_vehicle.rel_y
        new_dV = this_vehicle.rel_vy - v
        accel_if_change = this_vehicle.longitudinal_model.propagate(v, new_gap, new_dV)
        return accel_if_change > -self.b_safe

    def incentive_criterion(
            self,
            this_vehicle,
            back_vehicle,
            fore_vehicle,
            back_vehicles_fore_vehicle,
            ego_vy=15.0):
        """
        Returns a boolean for whether this vehicle moving into the target lane
            would satisfy the incentive criterion.
            In other words, would changing lanes be desired for our vehicle?

        Arguments
          this_vehicle:
            A vehicle object, representing the vehicle that we want to check
            if it should or should not change lanes.
          back_vehicle:
            A vehicle object, representing the vehicle that would be behind
            the ego vehicle if it changed lanes.
          fore_vehicle:
            A vehicle object, for the vehicle that is currently in front of our
            ego vehicle.
          back_vehicles_fore_vehicle:
            A vehicle object, for the vehicle that is in front of the vehicle
            that is behind us and in the target lane. See class-level comments.
          ego_vy:
            A float, indicating the absolute speed of the ego vehicle. This is
            necessary because the vehicle objects only contain relative speeds.

        Returns
          Boolean: True if we would like to change lanes.
        """
        back_accel_if_change = driver_model_utils.get_accel_y(
            this_vehicle,
            back_vehicle,
            ego_vy,
            this_vehicle.longitudinal_model)
        back_accel_no_change = driver_model_utils.get_accel_y(
            back_vehicles_fore_vehicle,
            back_vehicle,
            ego_vy,
            this_vehicle.longitudinal_model)

        my_accel_if_change = driver_model_utils.get_accel_y(
            back_vehicles_fore_vehicle,
            this_vehicle,
            ego_vy,
            this_vehicle.longitudinal_model)
        my_accel_no_change = driver_model_utils.get_accel_y(
            fore_vehicle,
            this_vehicle,
            ego_vy,
            this_vehicle.longitudinal_model)

        back_delta = back_accel_no_change - back_accel_if_change
        my_delta = my_accel_if_change - my_accel_no_change

        return my_delta > self.p * back_delta + self.a_thr

    def randomize_parameters(self, means, variances, debug=False):
        """
        Call to randomize the internal parameters of the driver model.
        Omiting a parameter name in either the mean or variance dictionary will
        simply result in the parameter not being randomized.
        Extraneous keys will be ignored, but will be printed if in debug mode.


        Arguments
          means: dictionary of `parameter_name` : `mean` for all parameters of
            the driver model that we would like to randomize. Omitting a
            parameter means we will not randomize it.
          variances: dictionary of `parameter_name` : `variance` for all parameters of
            the driver model that we would like to randomize. Omitting a
            parameter means we will not randomize it. As a reminder, standard
            deviation is the square root of the variance.
          debug: boolean for whether or not to print unusual situations.
        """
        for key in means.keys():
            if key not in variances:
                if debug:
                    print("Key {} appears in means for IDM,"
                          " but not variances.".format(key))
                continue
            if key is "p":
                self.p = np.random.normal(means[key], np.sqrt(variances[key]))
            elif key is "b_safe":
                self.T = np.random.normal(means[key], np.sqrt(variances[key]))
            elif key is "a_thr":
                self.s0 = np.random.normal(means[key], np.sqrt(variances[key]))
            elif key is "delta_b":
                self.a = np.random.normal(means[key], np.sqrt(variances[key]))
            elif debug:
                print("Invalid parameter to randomize (IDM): {}".format(key))
