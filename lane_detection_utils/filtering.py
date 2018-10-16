import math

########################################################################################################################################################
#
# Filtering
#
# Once the lanes have been generated we compare such lanes with previously generated lanes for the corresponding side (left or rith). If the deviation
# in terms of angle and distance with the lane generated on a previous frame is significant we discard the lane detected in the current frame and we
# assign as the current lane the one detected on a previous frame. The separation in distance is obtained by looking at the intersept of the lane and the 
# bottom of the image frame for both the current lane and for the one detected on th previous frame. In the horizontal distance between instercepts is too 
# high we discard the current detection.
#
########################################################################################################################################################

def filtering(count_lane_group1,count_lane_group2,x1_lane_group1,x1_lane_group2,H,initial_frame,mux_lane_vec_final1,muy_lane_vec_final1,base_ptx_lane_vec_final1,base_pty_lane_vec_final1,mux_lane_vec_final1_previous,muy_lane_vec_final1_previous,base_ptx_lane_vec_final1_previous,base_pty_lane_vec_final1_previous,mux_lane_vec_final2,muy_lane_vec_final2,base_ptx_lane_vec_final2,base_pty_lane_vec_final2,mux_lane_vec_final2_previous,muy_lane_vec_final2_previous,base_ptx_lane_vec_final2_previous,base_pty_lane_vec_final2_previous):

    if count_lane_group1>=1: 
        
        if initial_frame==1:
            angle1=180*(math.atan(muy_lane_vec_final1/mux_lane_vec_final1))/3.14159
            angle2=180*(math.atan(muy_lane_vec_final1_previous/mux_lane_vec_final1_previous))/3.14159
            
            if abs(angle1-angle2)>10:
                #("bad lane")
                mux_lane_vec_final1=mux_lane_vec_final1_previous
                muy_lane_vec_final1=muy_lane_vec_final1_previous
                base_ptx_lane_vec_final1=base_ptx_lane_vec_final1_previous
                base_pty_lane_vec_final1=base_pty_lane_vec_final1_previous
        
        #intersecting with top of image        
        Lintersection=-base_pty_lane_vec_final1/muy_lane_vec_final1
        x1_lane=base_ptx_lane_vec_final1+Lintersection*mux_lane_vec_final1
        #intersection with bottom of image
        Lintersection=(H-base_pty_lane_vec_final1)/muy_lane_vec_final1
        x2_lane=base_ptx_lane_vec_final1+Lintersection*mux_lane_vec_final1
        #intersection with bottom of image (for img3 the intersection needs to be different because the distance base_pty_lane_vec_final1 will be 1500+base_pty_lane_vec_final1 and this is closer to the bottom of the image than the previous case (we are using H=2160 in both cases, above it was a mistake still worked). So being closer to the bottom then x2_lane needs to be shorter.
        Lintersection2=(H-(1500+base_pty_lane_vec_final1))/muy_lane_vec_final1
        x2_lane2=base_ptx_lane_vec_final1+Lintersection2*mux_lane_vec_final1
        
        if initial_frame==1:
            if abs(x1_lane_group1-x1_lane)>50:
                mux_lane_vec_final1=mux_lane_vec_final1_previous
                muy_lane_vec_final1=muy_lane_vec_final1_previous
                base_ptx_lane_vec_final1=base_ptx_lane_vec_final1_previous
                base_pty_lane_vec_final1=base_pty_lane_vec_final1_previous
                Lintersection=-base_pty_lane_vec_final1/muy_lane_vec_final1
                x1_lane=base_ptx_lane_vec_final1+Lintersection*mux_lane_vec_final1
                Lintersection=(H-base_pty_lane_vec_final1)/muy_lane_vec_final1
                x2_lane=base_ptx_lane_vec_final1+Lintersection*mux_lane_vec_final1
            
       
        
        
        x1_lane_group1=x1_lane
        
        
        
    else:
        
        #intersecting with top of image        
        Lintersection=-base_pty_lane_vec_final1_previous/muy_lane_vec_final1_previous
        x1_lane=base_ptx_lane_vec_final1_previous+Lintersection*mux_lane_vec_final1_previous
        #intersection with bottom of image
        Lintersection=(H-base_pty_lane_vec_final1_previous)/muy_lane_vec_final1_previous
        x2_lane=base_ptx_lane_vec_final1_previous+Lintersection*mux_lane_vec_final1_previous
        
      
        mux_lane_vec_final1=mux_lane_vec_final1_previous
        muy_lane_vec_final1=muy_lane_vec_final1_previous
        base_ptx_lane_vec_final1=base_ptx_lane_vec_final1_previous
        base_pty_lane_vec_final1=base_pty_lane_vec_final1_previous
        
        x1_lane_group1=x1_lane
        
    
    if count_lane_group2>=1:
        if initial_frame==1:
            angle1=180*(math.atan(muy_lane_vec_final2/mux_lane_vec_final2))/3.14159
            angle2=180*(math.atan(muy_lane_vec_final2_previous/mux_lane_vec_final2_previous))/3.14159
            
            if abs(angle1-angle2)>10:
                #("bad lane")
                mux_lane_vec_final2=mux_lane_vec_final2_previous
                muy_lane_vec_final2=muy_lane_vec_final2_previous
                base_ptx_lane_vec_final2=base_ptx_lane_vec_final2_previous
                base_pty_lane_vec_final2=base_pty_lane_vec_final2_previous
        
        #intersecting with top of image        
        Lintersection=-base_pty_lane_vec_final2/muy_lane_vec_final2
        x1_lane=base_ptx_lane_vec_final2+Lintersection*mux_lane_vec_final2
        #intersection with bottom of image
        Lintersection=(H-base_pty_lane_vec_final2)/muy_lane_vec_final2
        x2_lane=base_ptx_lane_vec_final2+Lintersection*mux_lane_vec_final2
        #intersection with bottom of image (for img3 the intersection needs to be different because the distance base_pty_lane_vec_final1 will be 1500+base_pty_lane_vec_final1 and this is closer to the bottom of the image than the previous case (we are using H=2160 in both cases, above it was a mistake still worked). So being closer to the bottom then x2_lane needs to be shorter.
        Lintersection2=(H-(1500+base_pty_lane_vec_final2))/muy_lane_vec_final2
        x2_lane2=base_ptx_lane_vec_final2+Lintersection2*mux_lane_vec_final2
        
        if initial_frame==1:
            if abs(x1_lane_group2-x1_lane)>50:
                mux_lane_vec_final2=mux_lane_vec_final2_previous
                muy_lane_vec_final2=muy_lane_vec_final2_previous
                base_ptx_lane_vec_final2=base_ptx_lane_vec_final2_previous
                base_pty_lane_vec_final2=base_pty_lane_vec_final2_previous
                Lintersection=-base_pty_lane_vec_final2/muy_lane_vec_final2
                x1_lane=base_ptx_lane_vec_final2+Lintersection*mux_lane_vec_final2
                Lintersection=(H-base_pty_lane_vec_final2)/muy_lane_vec_final2
                x2_lane=base_ptx_lane_vec_final2+Lintersection*mux_lane_vec_final2
        
        x1_lane_group2=x1_lane
    
        
    else:
        #intersecting with top of image        
        Lintersection=-base_pty_lane_vec_final2_previous/muy_lane_vec_final2_previous
        x1_lane=base_ptx_lane_vec_final2_previous+Lintersection*mux_lane_vec_final2_previous
        #intersection with bottom of image
        Lintersection=(H-base_pty_lane_vec_final2_previous)/muy_lane_vec_final2_previous
        x2_lane=base_ptx_lane_vec_final2_previous+Lintersection*mux_lane_vec_final2_previous
        
        
        mux_lane_vec_final2=mux_lane_vec_final2_previous
        muy_lane_vec_final2=muy_lane_vec_final2_previous
        base_ptx_lane_vec_final2=base_ptx_lane_vec_final2_previous
        base_pty_lane_vec_final2=base_pty_lane_vec_final2_previous
        
        x1_lane_group2=x1_lane


    return mux_lane_vec_final1,muy_lane_vec_final1,base_ptx_lane_vec_final1,base_pty_lane_vec_final1,mux_lane_vec_final2,muy_lane_vec_final2,base_ptx_lane_vec_final2,base_pty_lane_vec_final2,x1_lane_group1,x1_lane_group2
