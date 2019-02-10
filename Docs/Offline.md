# MonoRARP

## Running Offline
In the course of research it will probably be necessary to run an offline version
of the system.
For example, we want to evaluate the effectiveness of the online system, which
means we need to compare it to some ground truth values.
However, there are not many automotive risk datasets, especially from video sources.
Thus, we want to create our own.
This document details how to run such a version, what changes must be made, and
justifications for most of the design decisions.

### Motivation
First, it is worth noting that what is referred to as the online system is easily
run offline (just running on a video file instead of a webcam source).
However, what we refer to as an offline version is actually a version of the system
that will produce outputs for each component of the system that can be used as
ground truth inputs or outputs.
For example, if we want to train a neural net using the state output instead of raw video,
and output the predicted risk, this system should produce a dataset that can support that.

Minor details for the system are that we have a different script to run it, called
`run_offline_risk_prediction.sh`, which also details some different components.
Since it is being run offline, we can afford to be more accurate and precise, but
not as efficient in some processes.
The most obvious example of this is using a slower neural network.

### Overall design tradeoffs
The main change we want to support is really just saving more outputs from each
component.
There are many different possible approaches to this, so we discuss the options here.
1. Distributed: Each class/component saves their own outputs.
   - Pro: Allows for easy "save" and "load" functionality for each class.
   - Con: Each class must share a global identifier for the frame.
   - Pro: Format for each class is independent of other classes, increasing flexibility.
2. Centralized: The main program requests the inputs and outputs from the classes.
   - Pro: Classes do not need a global identifier for each frame, only care about
   producing their outputs and communicating to the central authority.
   - Con: Central authority needs to be able to aggregate all information for a particular
   image frame. Without frame identifiers dispersed throughout, prevents threading.
   - Pro: Could pickle a `frameXXX.pkl` as output, which would contain all the data
   for the particular frame in something like a dictionary. This is good for loading.
   - Con: For loading, the central authority would have to communicate the loaded value
   to the desired components.

Another tradeoff is how often to save output.
1. At the end:
   - Pro: no need to worry about appending to a data file, making Pickling easy.
   - Con: Memory. Everything will have to be stored in memory. This is not an option.
2. Batched, a natural extension of the above:
   - Pro: Same as above, no need to worry about appending to a data file, making Pickling easy.
   - Con: Lots of questions:
      - How often do you batch?
      - Which components know how often we batch?
      - Within a batch, do the identifiers reset?
   - Note: Easier with centralized than distributed.
3. Every frame, new file for each frame:
   - Pro: Same benefit of no appending.
   - Con: Lots of files are generated. File I/Os are slow. This creates a lot of them.
   Even for running offline this should be a consideration.
   - Pro: Easy to know what file to load. Works in both centralized and decentralized setting.
4. Every now and then append to a file:
   - Pro: Do not need a consistent batching, as long as there is a consistent ID.
   - Pro: limited number of files generated.
   - Con: Either need to load file to memory or have a filetype that is appendable. Prevents Pickling.

### Final design
We opt to begin by implementing a (1) distributed offline version that saves
a new file as a `<component>/<global_frame_num>.pkl` every frame (3).
This can be easily extended to the batched case (2) if we end up with that need.
This should offer the most flexibility in terms of loading from individual
components for later online use cases as well.

### TODOs:
 * Compute metrics, make sure perfect if ground truth = predicted.
 * Modify the values predicted by the risk predictor to make them consistent with the theory.
 * Support a technique to imitate a lower FPS (to get more accurate comparison of techniques)
 * After this is complete, investigate better object detectors.
   - Fine tune some of the other pretrained COCO models on KITTI?
   - YOLO integration?

### Guidelines:
 * Modify the `run_offline_risk_prediction.sh` script to point to the appropriate
 video location (`SOURCE`), indicate offline mode (`OFFLINE='true'`), and indicate
 the directory to save the data outputs to (`RESULTS_SAVE_PATH`).


*This research was funded by The Allstate Corporation.*
