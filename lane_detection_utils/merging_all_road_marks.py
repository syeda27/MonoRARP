import numpy as np

#####################################################################################################################################
#
# Merging all the Road Marks
#
# At the end of the day we should end-up with two line representing the lanes (one for the left and one for the right), no
# matter how many road markings we have detected. This procedure merges the white road marks tat have been found over one
# side of the ego-vehicle by averaging the unit vectors of the white road mark directions. As we now any line is represented by its
# unit vector and a point that belongs to the line (which we call base point here). We achieve the merging by averaging the 
# base points of the white marks (we actually average the x and y coordinates of the base points).
#
####################################################################################################################################

def merging_all_road_marks(angle_lanes,count_tracked_lanes2,mux_lane_vec_aggregated,muy_lane_vec_aggregated,base_ptx_lane_vec_aggregated,base_pty_lane_vec_aggregated):

    #Merging all found lanes (without repetition) into two lanes
    #first pick the first lane
    print("count_tracked_lanes: ",count_tracked_lanes2)
    
    lane_group1=np.zeros(40) #one group of lanes with positive angle
    lane_group2=np.zeros(40) #one group of lanes with negative angle
    
    count_lane_group1=0
    count_lane_group2=0

    mux_lane_vec_final1=0
    muy_lane_vec_final1=0
    base_ptx_lane_vec_final1=0
    base_pty_lane_vec_final1=0

    mux_lane_vec_final2=0
    muy_lane_vec_final2=0
    base_ptx_lane_vec_final2=0
    base_pty_lane_vec_final2=0
    
    
    
    for lanes in range(0,count_tracked_lanes2):
        if angle_lanes[lanes]>0:
            lane_group1[count_lane_group1]=int(lanes) #storing indexes
            count_lane_group1=count_lane_group1+1
          
        else:
            lane_group2[count_lane_group2]=int(lanes)
            count_lane_group2=count_lane_group2+1

            
    if count_lane_group1>=2:
        
         mux_lane_vec_acc=0
         muy_lane_vec_acc=0
         base_ptx_lane_vec_acc=0
         base_pty_lane_vec_acc=0
        
         for k4 in range(0,count_lane_group1):
                
             mux_lane_vec_acc=mux_lane_vec_acc+mux_lane_vec_aggregated[int(lane_group1[k4])]
             muy_lane_vec_acc=muy_lane_vec_acc+muy_lane_vec_aggregated[int(lane_group1[k4])]
             base_ptx_lane_vec_acc=base_ptx_lane_vec_acc+base_ptx_lane_vec_aggregated[int(lane_group1[k4])]
             base_pty_lane_vec_acc=base_pty_lane_vec_acc+base_pty_lane_vec_aggregated[int(lane_group1[k4])]
             
         mux_lane_vec_final1=mux_lane_vec_acc/count_lane_group1 
         muy_lane_vec_final1=muy_lane_vec_acc/count_lane_group1
         base_ptx_lane_vec_final1=base_ptx_lane_vec_acc/count_lane_group1
         base_pty_lane_vec_final1=base_pty_lane_vec_acc/count_lane_group1
            
    else:

        if count_lane_group1==1:
            
            mux_lane_vec_final1=mux_lane_vec_aggregated[int(lane_group1[0])]
            muy_lane_vec_final1=muy_lane_vec_aggregated[int(lane_group1[0])]
            base_ptx_lane_vec_final1=base_ptx_lane_vec_aggregated[int(lane_group1[0])]
            base_pty_lane_vec_final1=base_pty_lane_vec_aggregated[int(lane_group1[0])]
        
        
        
        
            
    if count_lane_group2>=2:
        
         mux_lane_vec_acc=0
         muy_lane_vec_acc=0
         base_ptx_lane_vec_acc=0
         base_pty_lane_vec_acc=0
        
         for k4 in range(0,count_lane_group2):
                
             mux_lane_vec_acc=mux_lane_vec_acc+mux_lane_vec_aggregated[int(lane_group2[k4])]
             muy_lane_vec_acc=muy_lane_vec_acc+muy_lane_vec_aggregated[int(lane_group2[k4])]
             base_ptx_lane_vec_acc=base_ptx_lane_vec_acc+base_ptx_lane_vec_aggregated[int(lane_group2[k4])]
             base_pty_lane_vec_acc=base_pty_lane_vec_acc+base_pty_lane_vec_aggregated[int(lane_group2[k4])]
             
         mux_lane_vec_final2=mux_lane_vec_acc/count_lane_group2 
         muy_lane_vec_final2=muy_lane_vec_acc/count_lane_group2
         base_ptx_lane_vec_final2=base_ptx_lane_vec_acc/count_lane_group2
         base_pty_lane_vec_final2=base_pty_lane_vec_acc/count_lane_group2
            
    else:

        if count_lane_group2==1:
            
            mux_lane_vec_final2=mux_lane_vec_aggregated[int(lane_group2[0])]
            muy_lane_vec_final2=muy_lane_vec_aggregated[int(lane_group2[0])]
            base_ptx_lane_vec_final2=base_ptx_lane_vec_aggregated[int(lane_group2[0])]
            base_pty_lane_vec_final2=base_pty_lane_vec_aggregated[int(lane_group2[0])]

    return mux_lane_vec_final1,muy_lane_vec_final1,base_ptx_lane_vec_final1,base_pty_lane_vec_final1,mux_lane_vec_final2,muy_lane_vec_final2,base_ptx_lane_vec_final2,base_pty_lane_vec_final2,count_lane_group1,count_lane_group2
            
