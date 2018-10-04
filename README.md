# Real time driver risk estimation using object detection
### Integrated inputs include monocular video sources and GPS

## Install Instructions
* Nothing directly for this repo
* If the use of YOLO is desired, install darkflow and the correct changes to net/help.py and net/build.py.
    - see https://github.com/djp42/darkflow2
* Need tensorflow/models/research/object_detection installed.
    - see https://github.com/tensorflow/models/tree/master/research/object_detection
    - We currently use just the resnet101 faster RCNN trained on the Kitti dataset
        - You will need to download the pretrained model

## Running Guide
* If all you want to do is run it, the only file you need to care about is run_risk_prediction.sh
    - It is a bash script that handles numerous flags. 
* YOLO currently does not support all of the flags. 
    - https://pjreddie.com/darknet/yolo/
    - this is where we need to run darkflow's pipeline instead of run_risk_prediction.py. 
    - Preliminary tested showed it did not work as well, despite being much faster. It could have been due to some error or too high of a threshold, but reducing to 0.4 from 0.6 did not help,
* Most of the flags in the script should be self-explanatory or explained in a comment. 
* I recommend reading over the file before running it (as with any script) so that you know what is going on.
* A highlight is the SOURCE flag. It can point to a video file for openCV to read or to a webcam (marked 0 or 1...)
* Make sure to redirect the paths in the .sh script to the appropriate locations.
* The script's main goal is to run run_risk_prediction.py with the appropriate flags. It als handles GPS logging and some other things. 

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

# Improvements to make
This work is currently just a litle bit more than a minimum viable product. To that end, there are a lot of possible improvements to make. Below I have listed the ones I have thought of, in no particular order:
* Prioritize these improvements
* Incorporate Lane Information
    - Helen Jiang did a great job of working on this, so we still have to incorporate it into this setup somehow.
* Determine why OpenCV does not take in full FOV and quality input from the webcam
* Improve the tracking
    - It does bad on fast moving cars and some other cases.
* Improve the object detection. 
    - This can be done through improving either the speed or the performance. 
    - Maybe a better model will come online, or one of the other TF models works better.
    - We can also look into augmenting the pretrained model by fine tuning on another dataset.
    - A simple posibility is to preprocess the image better, through de-glaring and things like that.
* Incorporate horizon detection for improved generality in some of the distance calculations
    - This could help us determine the angle of the camera, but if it is 0 like we assume, horizon should be directly in the middle. 
* Improve the IDM and MOBIL parameters by basing it on the observed state for a vehicle instead of using general parameters for the entire scene
    - Behavioral parameter estimation is an open area of research I believe
* Improve IDM and MOBIL by adding an attentiveness parameter
* Develop an offline risk predictor based on the results we have here. 
    - A lot of possibilities here, including creating our own video - to - risk dataset, or just using the state information as the input
* Smarter tracker reinitialization
    - For example, when no cars are detected, no point using a tracker.
* Build a camera stand that allows easy verification of angles
    - Enlist one of my ME friends for this
* Smarter identification of objects
    - Would allow us to maintain state between tracker resets.
    - Simple approach is to track the box center between tracker resets and assume the closest together are the same object.
* When calculating distance, extend state object to include time so that the speed calculation is more accurate.
* Better incorporate yolo into the fold so its easier to do everything I do with non-yolo models with the yolo model as well. 
* Better tests, as always



*This research was funded by The Allstate Corporation.*
