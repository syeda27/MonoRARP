# Real time driver risk estimation using object detection
### Integrated inputs include monocular video sources and GPS

TODO: update all documents...

# Improvements to make
* Prioritize these improvements
* just wrap the detector in a class!
* Clean up organization a bit. Everything isnt a util... make a bin/ folder.
* Continue cleaning up code, especially tracker and lane detector.
* Improve the tracking
    - See file for in-line TODOs
    - pass in arguments
    - better motion model
    - smooth box dimensions over time
    - scale max box translation / likelihood by the box size.
    - visualize box movement as it is being held.
* Incorporate Lanes
    - something like lane-relative heading.
* Improve the object detection.
    - This can be done through improving either the speed or the performance.
    - Maybe a better model will come online, or one of the other TF models works better.
    - We can also look into augmenting the pretrained model by fine tuning on another dataset.
    - A simple possibility is to preprocess the image better, through de-glaring and things like that.
* Incorporate horizon detection for improved generality in some of the distance calculations
    - This could help us determine the angle of the camera, but if it is 0 like we assume, horizon should be directly in the middle.
* Improve the IDM and MOBIL parameters by basing it on the observed state for a vehicle instead of using general parameters for the entire scene
    - Behavioral parameter estimation is an open area of research I believe
* Improve IDM and MOBIL by adding an attentiveness parameter
* Develop an offline risk predictor based on the results we have here.
    - A lot of possibilities here, including creating our own video - to - risk dataset, or just using the state information as the input
* Smarter tracker reinitialization (KCF tracker)
    - For example, when no cars are detected, no point using a tracker.
* Build a camera stand that allows easy verification of angles
    - Enlist one of my Mech. Eng. friends for this
* Smarter identification of objects (KCF tracker)
    - Would allow us to maintain state between tracker resets.
    - Simple approach is to track the box center between tracker resets and assume the closest together are the same object.
* Better incorporate yolo into the fold so its easier to do everything I do with non-yolo models with the yolo model as well.
* Better tests, as always



*This research was funded by The Allstate Corporation.*
