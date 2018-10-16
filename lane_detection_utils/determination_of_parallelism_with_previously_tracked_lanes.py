import math

#############################################################################################################################################
#
# Determination of Paralelism and Distance
#
# It could be possible that for one reaason or another a white road mark may not satisfy all the criteria foor signature detection. Still
# the road amrking may be a valid one. We assess in this function wheather or not the white marking is sufficiently parallel to a previous
# tracked Lane. If the white mark is parallel and if its hoorizontal distance to the tracked lane is not too far then we re-qualify such
# white marking as a valid one later.
#
############################################################################################################################################
def determination_of_parallelism(top_left,rx1,ry1,count_lane_group1,count_lane_group2,whitemarkings_average,road1_average,base_ptx_lane_vec_final1,base_pty_lane_vec_final1,mux_lane_vec_final1,muy_lane_vec_final1,base_ptx_lane_vec_final2,base_pty_lane_vec_final2,mux_lane_vec_final2,muy_lane_vec_final2,angles):

    aligned_to_tracked_lane=0
                    
    if whitemarkings_average/road1_average>1:
        if count_lane_group1!=0 or count_lane_group2!=0:
                 
            for selection in range(0,2): #we compare against just two previous final lanes
                        
                proceed=0
                if selection==0 and count_lane_group1!=0:
                    proceed=1
                    xb=base_ptx_lane_vec_final1
                    yb=base_pty_lane_vec_final1
                    mux_previous_lane=mux_lane_vec_final1
                    muy_previous_lane=muy_lane_vec_final1
                                    
                if selection==1 and count_lane_group2!=0:
                    proceed=1
                    xb=base_ptx_lane_vec_final2
                    yb=base_pty_lane_vec_final2
                    mux_previous_lane=mux_lane_vec_final2
                    muy_previous_lane=muy_lane_vec_final2
                                    
                                       
                                    
                if proceed==1:    
                    if abs(angles[top_left]-180*(math.atan(muy_previous_lane/mux_previous_lane)/3.14159))<6:
                        
                                                             
                        x0=rx1[top_left]
                        y0=ry1[top_left]
                        Lproj=((x0-xb)*mux_previous_lane+(y0-yb)*muy_previous_lane)/((mux_previous_lane)**2+(muy_previous_lane)**2)
                                   
                        xp=xb+Lproj*mux_previous_lane
                        yp=yb+Lproj*muy_previous_lane
                            
                        distance_to_Lane=((x0-xp)**2+(y0-yp)**2)**0.5
                                       
                        if distance_to_Lane<10:
                            aligned_to_tracked_lane=1

    return aligned_to_tracked_lane
