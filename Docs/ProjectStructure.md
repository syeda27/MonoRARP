# Real time driver risk estimation using object detection
### Integrated inputs include monocular video sources and GPS

## File Breakdown
The repo is broken down into a variety of sub-components that I will call modules.

### Module 1: MAIN (run_risk_prediction.py)
This file is what I refer to as the main module. It handles all of the high level tasks such as reading in the image frames through openCV and displaying input, but for almost everything else it calls other modules.

It does handle some of the object detection things, but I try to limit that to the absolutely necessary.

I drew inspiration for this file from darkflow's demo file.
### Module 2: STATE (obj_det_state.py)
This file holds what I call the *state* for the risk estimation (a class). The state includes the ego speed and a dictionary that attempts to maintain the state parameters (relative distance and speed, both laterally and longitudinally) as long as possible, until they are reset, or reach a defined maximum number of states.

Another important component of this file is the state calculation. Not only does it store the state, but it also contains functions that calculate the relative distances and speed based on the bounding box inputs.
This process is pretty complicated and one of the todos is to do more testing to figure out how confident we are in these values.
We use a variety of methods to calculate the distances, averaging the results as a quick fix to get a more reliable result.
All I can say is, I am happy its within an order of magnitude right now.

It currently keys objects based on whatever is passed in, a number, but it could be extended to somehow make an ID of a vehicle based on the image bounding box. I think that should be handled in a different module though.
### Module 3: VEHICLE (vehicle.py)
One of the smaller modules, it is definitely not the least important.
Vehicle.py contains the vehicle class, which stores the current information about a vehicle on the road, including:
* the vehicle identifier
* the relative position and speed, both longitudinal and lateral
    - relative to the ego vehicle
* the longitudinal and lateral driver models, with given parameters (this will make more sense later)
The main function of the vehicle class is probably the get_action() function, which returns the longitudinal and lateral acceleration that the vehicle would take given the scene.
### Module 4: SCENE (scene.py)
The purpose of the scene class is to maintain a representation of all of the vehicles in a given position on the road.
This includes not just the list of the vehicles we know about, but also a dictionary of means and variances for the driver model parameters (things like politeness).
When we use the scene to simulate forward in time, we have the option of randomizing each of the vehicle paramters around these means and variances.

One of the more important functions of scene is finding neighboring cars. For both the IDM and MOBIL driver models (more on that later) we need too know some of the neighboring vehicles, and the scene class is where we find those.
This process is by no means efficient, but that can be optimized later.
### Module 5: DRIVER MODELS (driver_models.py)
This module contains a class for each supported driver model.
A driver model takes in a current vehicle and other necessary neighbor vehicles from the scene, and outputs an action.

At the time of writing, the current models supported are IDM[http://traffic-simulation.de/IDM.html] for longitudinal acceleration and MOBIL[http://traffic-simulation.de/MOBIL.html] for lateral acceleration and lane changes (MOBIL actually depends on the IDM acceleration).

The driver models crucially support randomizing their parameters given a dictionary of means and variances.
### MODULE 6: RISK (risk_pred.py)
This is where we finally get some interesting numbers for the driver.
Here, we use all of the previous modules to run N simulations given the initial STATE extracted from the image/video by creating a SCENE which allows us to simulate forward based on the DRIVER MODELS and VEHICLES.

The class *risk_predictor* is designed to be maintained globally in the MAIN module. With each update to the STATE, we can call get_risk(), which will return a number (between 0 and 10) that we call our automotive risk.

This risk value is calculated by taking evaluating each rollout for the number of collisions (risk score of 10) and low time-to-collision events (risk score of 1) that include the ego vehicle, which are averaged over all scenes and then we take the average risk over all rollouts. For details, see calculate_risk()

### MODULE 7: GPS (use_gps.py)
I should probably rename this file... However, its purpose is simple: read and parse the file the gps device is logging to in order to calculate the absolute speed of the ego vehicle over the last second. There are some details there about how to parse the file - it currently only supprots NMEA format - but the main idea is simple.

This module is called directly from the MAIN module I believe. That may not be optimal, but it seems to work so far.

## Tests
I have a *tests* folder which includes some test scripts. I would not call them entirely robust as of yet, but some of the modules are a lot harder to test than others. The GPS testing is probably the best one.

For things like the STATE, instead of testing as its normally defined I have that *calibrations* folder, which is where I was printing out distance calculations in known setups. I do think I need to repeat this at some point with a better setup, the original test was in a parking lot that I do not think was entirely flat...


*This research was funded by The Allstate Corporation.*
