import cv2
import math
import numpy as np

"""
Scan Region

Author: Juan Carlos Aragon - Allstate
Minor Editing: djp42 - Stanford

See README for more details.
We scan the image with a rectangular region 120 by 160 so that we can analyze the lines inside the region
"""

def scan_region(dlines, xregion, yregion, region_length, region_width):
    #Initializing vectors to be used for collection of the lines inside the region
    rx1 = np.zeros(100)
    rx2 = np.zeros(100)
    ry1 = np.zeros(100)
    ry2 = np.zeros(100)
    #Initializing vector for angle collection
    angles = np.zeros(100)
    count_angles_per_region = 0
    #For all the lines detected in the image subframe we collect each line that falls inside the scanning region and we analize it.
    for dline in dlines[0]:
        x0 = int(round(dline[0][0]))
        y0 = int(round(dline[0][1]))
        x1 = int(round(dline[0][2]))
        y1 = int(round(dline[0][3]))
        if x0 > xregion - int(region_width / 2) and \
                x0 < xregion + int(region_width / 2) and \
                y0 > yregion - int(region_length / 2) and \
                y0 < yregion + int(region_length / 2) and \
                x1 > xregion - int(region_width / 2) and \
                x1 < xregion + int(region_width / 2) and \
                y1 > yregion - int(region_length / 2) and \
                y1 < yregion + int(region_length / 2):
            #we establish a convention for each line where pt1 is the line's end-point that is in top. Thus pt2 is below pt1.
            if y1 > y0:
                pt1_y = y0
                pt2_y = y1
                pt1_x = x0
                pt2_x = x1
            else:
                pt1_y = y1
                pt2_y = y0
                pt1_x = x1
                pt2_x = x0

            #we generate arrays that allow to correlate each line's end-point coordinates with the line's angle. The angle will be used as filtering criteria below.
            if pt2_x != pt1_x:  #we rearely have pt1_x =  = pt2_x due to perspective. The exception will be a lane changing maneuver (in this case for a moment the lines corresponding to white markings become vertical. In that case we will have a temporal exception that nontheless should not impact performance)
                angle = 180*(math.atan((pt2_y-pt1_y) / (pt2_x-pt1_x))) / 3.14159
                if abs(angle) > 20 and abs(angle) < 70:
                    angles[count_angles_per_region] = angle
                    rx1[count_angles_per_region] = pt1_x
                    rx2[count_angles_per_region] = pt2_x
                    ry1[count_angles_per_region] = pt1_y
                    ry2[count_angles_per_region] = pt2_y
                    count_angles_per_region += 1

    return rx1, rx2, ry1, ry2, angles, count_angles_per_region
