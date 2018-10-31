import cv2

########################################################################################################################################################################################################
#
# ABSOLUTE SPEED ESTIMATION
#
# This procedure relies on a "marker" which is a small imaginary horizontal line that is used as reference to establish speed. The Y-coordinate of this "marker" is fixed at a value of 
# 0.8*h1 where h1 is the height of the image subframe used to perform Lane Detection and speed estimation. The procedure below uses as input the unit vector of the line representing the Lane  
# and a base point for this line. The first step is to determine the intersection between the line representing the Lane and a horizontal line that has the Y-coordinate of the "marker". This
# intersection becomes the X-coordinate of the marker's centroid and we call this value as "x2_lane" below. We proceedto scan horizontally the brightness of the pixels around the "marker's centroid
# If a high transition on brightness is detected as a result of the scan we have detected that the marker has touched the beginning of the white road marking. Since the vehicle moves forward
# we mention beginning because the progression of image frames will make the marker "move" over the white road marking from one end to the other. In this sense "beginning" corresponds to the
# event when the marker is in contact with a given white road marking for the first time.
#
# We set a time-stamp every time this high transition is detected after of period of time when no high transitions were happening (which points to a situation where the "marker" was over the pavement). # # Therefore we expect two consecutive time-stamps. One time stamp at the begiining of the white road marking and another at the beggining of the next white road marking.
#
# At every time-stamp it could have happended that the marker is way past the beggining of the white road marking, in which case we scan downwards to find the actual beggining of the white road marking 
# and we use this information to compute a correction factor on the distance that the car traveled (since the time stamp set at this moment won't correspond to exactly 40  feet). The correction is 
# performed for both time-stamp events.
#
#########################################################################################################################################################################################################

def absolute_speed_estimation(muy_lane_vec_speed,mux_lane_vec_speed,base_ptx_lane_vec_speed,base_pty_lane_vec_speed,h1,capture_frameindex_for_speed,frameindex_for_speed_previous,frameindex_for_speed,image_number,white_mark_hit,count_scanned_lines_reverse_for_speed_previous,count_scanned_lines_reverse_for_speed,img6,img2):
    
 
    
    speed_read_flag=0
    speed=0
        
    #Intersection between the Lane and the marker
    Lintersection=(0.8*h1-base_pty_lane_vec_speed)/muy_lane_vec_speed
    x2_lane=base_ptx_lane_vec_speed+Lintersection*mux_lane_vec_speed
        
    cv2.line(img6, (int(x2_lane-50), int(0.8*h1)), (int(x2_lane+50), int(0.8*h1)), (255,0,0), 1, cv2.LINE_AA)
        
    #Scanning horizontally over the marker and detect high transition
    road_nomark=1
    for k5 in range(-15,15):
            
        value = img2[int(0.8*h1),int(x2_lane+k5)]
            
        if k5>-15:
            if value>1.2*previous:
                road_nomark=0    
                
            
                        
                #scanning upwards to find the end of the mark
                found_end_of_mark=0
                count_scanned_lines=0
                for k6 in range(0,100):
                    Lintersection=(0.8*h1-k6-base_pty_lane_vec_speed)/muy_lane_vec_speed
                    x2_lane_scan=base_ptx_lane_vec_speed+Lintersection*mux_lane_vec_speed
                        
                    cv2.line(img6, (int(x2_lane_scan-50), int(0.8*h1-k6)), (int(x2_lane_scan+50), int(0.8*h1-k6)), (255,0,0), 1, cv2.LINE_AA)
                    count_scanned_lines=count_scanned_lines+1
                        
                    out_of_the_mark=1
                    for k7 in range(-10,10):
                            
                        value_scan = img2[int(0.8*h1-k6),int(x2_lane_scan+k7)]
                        if k7>-10:
                            if value_scan>1.2*previous_scan:
                                out_of_the_mark=0
                                break
                                
                        previous_scan=value_scan
                            
                    if out_of_the_mark==1:
                        break
                        
                        
                #finding the beginning of the lane (we scan in reverse when the mark is not set properly at beggining of road mark. Reverse direction is scanning downwards)      
                found_end_of_mark_reverse=0        
                count_scanned_lines_reverse=0
                for k6 in range(0,100):
                    Lintersection=(0.8*h1+k6-base_pty_lane_vec_speed)/muy_lane_vec_speed
                    x2_lane_scan=base_ptx_lane_vec_speed+Lintersection*mux_lane_vec_speed
                        
                    cv2.line(img6, (int(x2_lane_scan-50), int(0.8*h1+k6)), (int(x2_lane_scan+50), int(0.8*h1+k6)), (0,0,255), 1, cv2.LINE_AA)                  
                    count_scanned_lines_reverse=count_scanned_lines_reverse+1
                        
                    out_of_the_mark_reverse=1
                    for k7 in range(-20,20):
                        
                        if 0.8*h1+k6>=h1:
                            break
                        else:    
                            value_scan = img2[int(0.8*h1+k6),int(x2_lane_scan+k7)]
                        
                        if k7>-20:
                            if value_scan>1.2*previous_scan:
                                out_of_the_mark_reverse=0
                                break
                                
                        previous_scan=value_scan
                            
                    if out_of_the_mark_reverse==1:
                        break        
                
                if capture_frameindex_for_speed==0 and count_scanned_lines>=20:
                    white_mark_hit=1
                    frameindex_for_speed=image_number
                    count_scanned_lines_reverse_for_speed=count_scanned_lines_reverse
                    capture_frameindex_for_speed=1
                    
                    if frameindex_for_speed_previous!=0:
                        time=(frameindex_for_speed-frameindex_for_speed_previous)/29
                        speed=(40/time)*0.682
                        speed_read_flag=1
                        
                        
                        
                        if count_scanned_lines+count_scanned_lines_reverse>40: #meaning correct road mark
                            correction_factor=(count_scanned_lines_reverse/115)*10 #excess of distance traveled beyond the beginning of the white road marking. The "10" here is the 10 feet of the road marking
                            correction_factor2=(count_scanned_lines_reverse_for_speed_previous/115)*10
                            speed=((40-correction_factor-correction_factor2)/time)*0.682

                   
                
                break
        previous=value



    return speed,road_nomark,capture_frameindex_for_speed,frameindex_for_speed,white_mark_hit,speed_read_flag,count_scanned_lines_reverse_for_speed
        
    

        
