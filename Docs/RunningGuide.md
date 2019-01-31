# MonoRARP

## Running Guide
* If all you want to do is run it, the only file you need to care about is run_risk_prediction.sh
    - It is a bash script that handles numerous flags.
* Most of the flags in the script should be self-explanatory or explained in a comment.
* I recommend reading over the file before running it (as with any script) so that you know what is going on.
* A highlight is the SOURCE flag. It can point to a video file for openCV to read or to a webcam (marked 0 or 1...)
* Make sure to redirect the paths in the .sh script to the appropriate locations.
    - pass in the path to your local tensorflow object detection api location as the first positional argument to the bash script.
* The script's main goal is to run launcher.py with the appropriate flags. It also handles GPS logging and some other things.
* To use the speed_estimator based on lane markings, see [../speed_estimator_utils/README.md](../speed_estimator_utils/README.md) for details.

### C++ (WORK IN PROGRESS)
* To compile and run in C++:
    - `./compile_cpp.sh`  <-- this cleans all of the exisiting CMake files and compiled binaries,and then runs `cmake` and `make` in the `Compile` directory.
    - Then run the binaries in the `bin` file.
    - This is nowhere near complete, do not expect it to work.

*This research was funded by The Allstate Corporation.*
