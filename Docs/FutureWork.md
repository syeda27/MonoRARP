# MonoRARP

## Improvements to make
* Better tests, as always
* Clean up organization a bit. Everything isnt a util... make a bin/ folder.
* Continue cleaning up code, especially tracker and lane detector.
* Improve the tracking
    - See file for in-line `TODOs`
    - Use a better motion model besides constant velocity.
        - Using the vehicle state objects would be ideal, transforming from the image frame to the world frame and back. However, due to computational constraints we should exercise caution with that approach.
    - Smooth box dimensions over time, potentially with a Kalman Filter, etc.
    - Scale max box translation / likelihood by the box size.
    - Visualize box movement as it is being held.
        - We currently do not show the movement (constant velocity in image frame) of held tracking boxes, but visualizing it may help with debugging and improvements.
* Incorporate Lanes
    - A useful feature would be lane-relative heading.
* Improve the object detection.
    - This can be done through improving either the speed or the performance.
    - Maybe a better model will come online, or one of the other TF models works better than FasterRCNN.
    - We can also look into augmenting the pretrained model by fine tuning on another dataset.
    - A simple possibility is to preprocess the image better, through de-glaring and things like that.
    - Finish incorporating and testing YOLO.
* Incorporate horizon detection for improved generality in some of the distance calculations
    - This could help us determine the angle of the camera, but if it is 0 like we assume, horizon should be directly in the middle.
* Improve the IDM and MOBIL parameters by basing it on the observed state for a vehicle instead of using general parameters for the entire scene
    - Behavioral parameter estimation is an open area of research.
* Improve IDM and MOBIL by adding an attentiveness parameter
* Develop an offline risk predictor based on the results we have here.
    - A lot of possibilities here, including creating our own video - to - risk dataset, or just using the state information as the input
* Build or buy a camera stand that allows easy verification of angles
* Convert more and more of the code to C++ to speed up runtime.
* Experiment with converting to a ROS-compatible setup.



*This research was funded by The Allstate Corporation.*
