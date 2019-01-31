# MonoRARP
## File Breakdown
The repo is broken down into a variety of sub-components that I will call modules. It can get complicated at times, so this docuement is designed to help sort through the mess.

### Launcher: (launcher.py)
This file sets up the Tensorflow framework and calls the respective Runner or ThreadedRunner, depending on the arguments.

### Runner / ThreadedRunner: (runner.py, threaded_runner.py)
These modules are tasked with most of the coordination of the high level tasks such as reading in the image frames through openCV and displaying input, but for almost everything else it calls other modules.

The Runner handles some of the object detection tasks.

### State History (state_history.py)
This holds a class with the simple task of maintaining states associated with vehicles over time.
The main functionality is to update the state based on a given bounding box and key.
In addition to updating vehicle states and maintaining that history, this module also stores an ego_speed value.
This interfaces with the state_estimation_utils to actually calculate state values for detected objects, such as relative distance and speed, which are critical components of the system.

### Vehicle and Vehicle State (vehicle.py, vehicle_state.py)
One of the smaller modules, it is definitely not the least important.
Vehicle.py contains the vehicle class, which stores the current information about a vehicle on the road, including:
* the vehicle identifier
* the relative position and speed, both longitudinal and lateral
    - relative to the ego vehicle
* the longitudinal and lateral driver models, with given parameters
The main function of the vehicle class is the get_action() function, which returns the longitudinal and lateral acceleration that the vehicle would take given the scene.
VehicleState is a simple wrapper to track specific quantities and the last update time.

### Scene (scene.py)
The purpose of the scene class is to maintain a representation of all of the vehicles in a given position on the road, and support simulating these *vehicles* forward in time.
This includes not just the list of the vehicles we know about, but also a dictionary of means and variances for the driver model parameters (things like politeness).
When we use the scene to simulate forward in time, we have the option of randomizing each of the vehicle parameters around these means and variances.

One of the more important functions of scene is finding neighboring cars. For both the IDM and MOBIL driver models we need too know some of the neighboring vehicles, and the scene class is where we find those.

### Driver Models (driver_models.py)
This module contains a class for each supported driver model.
A driver model takes in a current vehicle and other necessary neighbor vehicles from the scene, and outputs an action.

At the time of writing, the current models supported are IDM[http://traffic-simulation.de/IDM.html] for longitudinal acceleration and MOBIL[http://traffic-simulation.de/MOBIL.html] for lateral acceleration and lane changes (MOBIL actually depends on the IDM acceleration).

The driver models crucially support randomizing their parameters given a dictionary of means and variances.

### Risk Predictor (risk_predictor.py)
This module interfaces with the Scene and StateHistory to produce risk values. The method is user-specified, including simple time-to-collision using constant velocity, and the more complicated process of using the scene to simulate a certain number of potential evolutionary rollouts into the future, and aggregating the risk seen in those rollouts.

#### Embedded Risk Predictor (embedded_risk_predictor.py)
The embedded_risk_predictor combines the RiskPredictor and Scene to avoid copying all of the scene rollouts together. Instead, as we simulate potential paths forward, we maintain the desired risk score and update it with each new scene. This is much more efficient from a runtime perspective, but limits the modularity, so we maintain both options.

### Speed Estimator (speed_estimator.py)
This is tasked with calculating the speed of the ego vehicle. It supports multiple options, which are user specified.

### Tracker (tracker.py)
Perhaps the most complicated module, the tracker is really a Multi-Object tracker, as defined in multi_trackers.py.
Multiple different types of trackers are supported, which are also user specified, although the KCF multiobject tracker has many limitations.
For more details I recommend starting at tracker.py and looking through the code and subclasses.
The main idea is to take in an image and potentially bounding boxes output by an object detection network, and tracking the objects over time.

### Object Detector (object_detector.py)
With the threaded_runner, we are able to easily separate the obejct detector into its own class object.
This will help with the substitution of additional object detection networks to compare to, as long as the input/output of the classes is the same, which is defined in the abstract object_detector class.

### Lane Detector (lane_detector.py) - WORK IN PROGRESS
This is an attempt to detect the lane markings on the road, and does not currently work. See `lane_detection_utils` and the associated `README` for details.


## Tests
I have a *tests* folder which includes some test scripts. I would not call them entirely robust as of yet, but some of the modules are a lot harder to test than others.

For things like the STATE, instead of testing as its normally defined I have that *calibrations* folder, which is where I was printing out distance calculations in known setups. I do think I need to repeat this at some point with a better setup, the original test was in a parking lot that I do not think was entirely flat...


*This research was funded by The Allstate Corporation.*
