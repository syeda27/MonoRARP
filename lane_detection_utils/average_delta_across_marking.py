
"""
Average Delta
Author: Juan Carlos Aragon - Allstate
Minor Editing: djp42 - Stanford

See README for more details.
This is a measure of dispersion for each sample type within the scanning region.
"""

import cv2

def average_delta(road1_vec,
                  road2_vec,
                  road3_vec,
                  road4_vec,
                  whitemarkings_vec,
                  road1_average,
                  road2_average,
                  road3_average,
                  road4_average,
                  whitemarkings_average,
                  removed3,
                  removed4,
                  counter_scanning):

    #Average Delta between the brigthness of each sampled pixel and the mean of the brigthness across the horizontal scanning (for each of the 5 types of sample pixels)
    delta_road1_accumulator = 0
    delta_road2_accumulator = 0
    delta_road3_accumulator = 0
    delta_road4_accumulator = 0
    delta_whitemarkings_accumulator = 0

    for j1 in range(0, counter_scanning):
        delta_road1_accumulator = delta_road1_accumulator + \
                                  abs(road1_vec[j1] - road1_average)
        delta_road2_accumulator = delta_road2_accumulator + \
                                  abs(road2_vec[j1] - road2_average)
        if removed3 == 0 or (removed3 != j1):
            delta_road3_accumulator = delta_road3_accumulator + \
                                      abs(road3_vec[j1] - road3_average)
        if removed4 == 0 or (removed4 != j1):
            delta_road4_accumulator = delta_road4_accumulator + \
                                      abs(road4_vec[j1] - road4_average)
        delta_whitemarkings_accumulator = delta_whitemarkings_accumulator + \
                                      abs(whitemarkings_vec[j1] -
                                          whitemarkings_average)


    delta_road1_average = delta_road1_accumulator / counter_scanning
    delta_road2_average = delta_road2_accumulator / counter_scanning
    if removed3 == 0:
        delta_road3_average = delta_road3_accumulator / counter_scanning
    else:
        delta_road3_average = delta_road3_accumulator / (counter_scanning - 1)
    if removed4 == 0:
        delta_road4_average = delta_road4_accumulator / counter_scanning
    else:
        delta_road4_average = delta_road4_accumulator/(counter_scanning - 1)
    delta_whitemarkings_average = delta_whitemarkings_accumulator / \
                                  counter_scanning

    return  delta_road1_average, \
            delta_road2_average, \
            delta_road3_average, \
            delta_road4_average, \
            delta_whitemarkings_average
