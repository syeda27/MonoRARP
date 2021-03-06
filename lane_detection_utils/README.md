**There were many paragraph-like descriptions of the lane detection files, so I
moved them here to keep everything ~concise in the code. - djp42**
Author: Juan Carlos Aragon - Allstate

# ABSOLUTE SPEED ESTIMATION

This procedure relies on a "marker" which is a small imaginary horizontal line that is used as reference to establish speed. The Y-coordinate of this "marker" is fixed at a value of
0.8*h1 where h1 is the height of the image subframe used to perform Lane Detection and speed estimation. The procedure below uses as input the unit vector of the line representing the Lane
and a base point for this line. The first step is to determine the intersection between the line representing the Lane and a horizontal line that has the Y-coordinate of the "marker". This
intersection becomes the X-coordinate of the marker's centroid and we call this value as "x2_lane" below. We proceedto scan horizontally the brightness of the pixels around the "marker's centroid
If a high transition on brightness is detected as a result of the scan we have detected that the marker has touched the beginning of the white road marking. Since the vehicle moves forward
we mention beginning because the progression of image frames will make the marker "move" over the white road marking from one end to the other. In this sense "beginning" corresponds to the
event when the marker is in contact with a given white road marking for the first time.

We set a time-stamp every time this high transition is detected after of period of time when no high transitions were happening (which points to a situation where the "marker" was over the pavement). # # Therefore we expect two consecutive time-stamps. One time stamp at the begiining of the white road marking and another at the beggining of the next white road marking.

At every time-stamp it could have happended that the marker is way past the beggining of the white road marking, in which case we scan downwards to find the actual beggining of the white road marking
and we use this information to compute a correction factor on the distance that the car traveled (since the time stamp set at this moment won't correspond to exactly 40  feet). The correction is
performed for both time-stamp events.

# Average Delta
This is a measure of dispersion for each sample type within the scanning region. We take the average distance (delta) between each sample and
its corresponding mean (average) which was computed by the computed for all the "outlayer_removal_and_average_brightness_computation" function.
The average delta should be low enough in all cases if we are dealing with a white road marking because the areas on the marking tend to be uniform.

# Determination of Paralelism and Distance
It could be possible that for one reaason or another a white road mark may not satisfy all the criteria foor signature detection. Still
the road amrking may be a valid one. We assess in this function wheather or not the white marking is sufficiently parallel to a previous
tracked Lane. If the white mark is parallel and if its hoorizontal distance to the tracked lane is not too far then we re-qualify such
white marking as a valid one later.

# Eliminate Duplicate Road marks
Tho position of the scanning region could capture the same white road mark due to the size of the region and the step for the scanning.
We remove the duplicates generated in this way to proceed with a cleaner processing on the subsequent steps.

# Filtering
Once the lanes have been generated we compare such lanes with previously generated lanes for the corresponding side (left or rith). If the deviation
in terms of angle and distance with the lane generated on a previous frame is significant we discard the lane detected in the current frame and we
assign as the current lane the one detected on a previous frame. The separation in distance is obtained by looking at the intersept of the lane and the
bottom of the image frame for both the current lane and for the one detected on th previous frame. In the horizontal distance between instercepts is too
high we discard the current detection.

# Signature Detection
Due to uniformity (in terms of brightness) of the white road mark and the uniformity of the pavement around the white road mark
we assess if the statistics collected over the sampled brighness around the potential white road mark reflect this reality. The
satistics assess if the dispersion per type of sample is low enough. Similarly the actual average brigthness of the pavement for the
two types of samples collected over the left should be similar enough (same criteria is used over the samples collected over the right).
The actual white road mark brightness average should be distinctive enough over the pavement. We have used relative meassures in this case
which provides enhanced invariance to absolute variations in brightness across the whole image.

# Long Term Average
We perform averaging of the unit vectors and of the coordinates of the base ponts for the previous 6 lanes detections performed
Abrupt transient behavior is de-emphasized through this averaging which provides greater smoothness to the detection.

# Merging all the Road Marks
At the end of the day we should end-up with two line representing the lanes (one for the left and one for the right), no
matter how many road markings we have detected. This procedure merges the white road marks tat have been found over one
side of the ego-vehicle by averaging the unit vectors of the white road mark directions. As we now any line is represented by its
unit vector and a point that belongs to the line (which we call base point here). We achieve the merging by averaging the
base points of the white marks (we actually average the x and y coordinates of the base points).

# Outlayer Removal
On the pixels that are father away from the white road marking du to artifacts there are very high brightness even though these pixels
are supossed to be over the road and should be dark. The procedure in this script allows the removal of one artifact per detection.

# Average Brightness
For each type of sample we collect all the samples recorded vertically along the border of the road marking and in each case we take the average.
This will be useful later to determine if the signature for the white road mark exists.

# Brightness Sampling
Horizontal Lines that cross the two top-line segments are used to perform sampling. We use the unit vector of the
direction of the line and one end-point of this line to determine its intercept with the horizontal line. We sample
pixels around this intercept as described below.

# Scan Region
We scan the image with a rectangular region 120 by 160 so that we can analyze the lines inside the region
The variables xregion and yregion below are the X-Y coordinates of the rectangular region's centroid. For each line
we obtain the coordinates of the line's end-points and the line's angle.


# LANE DETECTION
The python script below delivers lane detection based on the LSD (line segment detector) algorithm and its application to the detection of white road markings. The main idea is to sample the
brightness of the pixels around the borders of the potential white road mark and extract specific charcateristics/features of the pixels arranged around. Based on these features a particular
signature for a white road mark is detected. Based on the detected white road marks a line representing the lane is determined by finding the line that crosses all the detected white marks on each
side (either left or right).

The script is divided into sections which we will describe below:

## I) IMAGE READING
 The image is read in grayscle for processing and in color for visualization. We take the bottom portion of the image (subframe) for LSD processing since this is the area
 of interest to find white markings.

## II) LSD ALGORITHM
We invoke the LSD algorithm to detect all straight line segments detected on the image. We redraw the detected lines in the image to emphasize the edges for subsequent processing.

## III) SCANNING OF THE IMAGE AND DETECTION OF ROAD MARK SIGNATURE
This section is subdivided as follows:
 a) Scan region and generate line segments
 We use a rectangular window that moves over the image subframe and we collect all the line segments inside the region for further analysis.
 b) From all the lines collected in a) we find the two lines that are closest to the top of the scanning region. These two lines will be used to find the signature.
 c) For the two-top lines found in b) we sample 5 pixels around the two lines. We use horizontal lines that cross the two-top lines and along each horizontal line we sample two pixels
 to the left of the left-line, two pixels to the right of the right-line and we sample one pixel that is in the middle between the two top-lines (the right-line and the left-line). We
 extract aggregated statistics based on the samples obtained. These statistics provide the ability to determine the existence of the white road mark signature if it exists.

## IV) ELIMINATION OF WHITE ROAD MARK DUPLICATES
Since the scanning region moves by steps it is possible that a white road marking could be captured more than once by two different positions of the scanning region. Here we eliminate duplicate
detections of the same white road marking.

## V) MERGING ALL WHITE ROAD MARKS PREVIOUSLY DETECTED AND GENERATION OF TWO LANES
Two o more white road markings detected over the left side of the ego-vehicle are used to determine the line that crosses all of them. This line becomes representative of the Lane defined by
these white road markings. Similarly the same processing is performed over the right side of the ego-vehicle.

## VI) FILTERING WHITE ROAD MARKS TO REMOVE FALSE DETECTIONS AND CORRECTION OF THE TWO LANES
If we compare the Lane that has been generated by white road marks in the current frame with the Lane that has been obatined in previous frames and we observe a significant deviation in terms of angle
and horizontal separation we deem the Lane as a false detection and we assign as the current Lane the Lane that was detected on the previous image frame.

## VII) RECORDING OF CURRENTLY DETECTED LANES
We keep a record of the two Lanes detected in the current frame so that we can use them for comparison/validation of future detections.

## VIII) GENERATION OF LONG TERM AVERAGE OF THE DETECTED LANES
We generate a long term average of Lanes detected over the last 6 frames in order to provide stability to the detection and dampen transients.

## IX) DISPLAY
We display the image emphasizing the area of the road with the white markings. This causes an aspect ration distortion overall, however the details of the detection are clearly visible.
