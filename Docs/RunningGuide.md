# Real time driver risk estimation using object detection
### Integrated inputs include monocular video sources and GPS

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
    - pass in the path to your local tensorflow object detection api location as the first positional argument to the bash script.
* The script's main goal is to run run_risk_prediction.py with the appropriate flags. It als handles GPS logging and some other things.

* To compile and run in C++ (IN PROGRESS):
    - `./compile_cpp.sh`  <-- this cleans all of the exisiting CMake files and compiled binaries,and then runs `cmake` and `make` in the `Compile` directory.
    - Then run the binaries in the `bin` file.


*This research was funded by The Allstate Corporation.*
