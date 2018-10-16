#######################################################################################################################################
#
# Outlayer Removal
# On the pixels that are father away from the white road marking du to artifacts there are very high brightness even though these pixels
# are supossed to be over the road and should be dark. The procedure in this script allows the removal of one artifact per detection.
# 
# Average Brightness
# For each type of sample we collect all the samples recorded vertically along the border of the road marking and in each case we take the average. 
# This will be useful later to determine if the signature for the white road mark exists.
#
########################################################################################################################################

def outlayer_removal_and_average_brightness_computation(road1_vec,road2_vec,road3_vec,road4_vec,whitemarkings_vec,counter_scanning):

                    
    #defining accumulators to obatin statistics over brightsness
    road1_acummulator=0
    road2_acummulator=0
    road3_acummulator=0
    road4_acummulator=0
    whitemarkings_acummulator=0



    #### Below we generate some statistics in order to qualify the signature that we are extracting for the segments                                                     #####
    #### The signature for the white road marking should comply with some criteria applied to the statistics of the samples extracted from the pixels around the marking #####
                

    #Outlayer Removal and Average Brightness Computation

    #Outlayer Removal
    #We take the two samples that are the farthest from the top-left line and the top-right line and we detect and remove one outlayer (pixel value that due to artifact is unsually high)
                
    removed1=0
    removed2=0
    removed3=0
    removed4=0
                
    for j1 in range(0,counter_scanning):
        #removing only one outlayer (this could be extended to remove more than one outlayer)
        outlayer1=0
        outlayer2=0
        outlayer3=0
        outlayer4=0
                    
        if j1>=1:
            if abs(road3_vec[j1]-current_average3)>10 and removed3==0:
                outlayer3=1
                removed3=j1
            if abs(road4_vec[j1]-current_average4)>10 and removed4==0:
                outlayer4=1
                removed4=j1
                    
        if outlayer3==0:
            if removed3==0:
                current_average3=(road3_acummulator+road3_vec[j1])/(j1+1)
            road3_acummulator=road3_acummulator+road3_vec[j1]  
                        
                            
        if outlayer4==0:
            if removed4==0:
                current_average4=(road4_acummulator+road4_vec[j1])/(j1+1)
            road4_acummulator=road4_acummulator+road4_vec[j1]  
                        
                    
        road1_acummulator=road1_acummulator+road1_vec[j1]
        road2_acummulator=road2_acummulator+road2_vec[j1]
        whitemarkings_acummulator=whitemarkings_acummulator+whitemarkings_vec[j1]



    #Average Brightness for the 5 samples taken around the two top lines on the scanning region
    road1_average=road1_acummulator/counter_scanning
    road2_average=road2_acummulator/counter_scanning

    if removed3==0:
        road3_average=road3_acummulator/counter_scanning
    else:
        road3_average=road3_acummulator/(counter_scanning-1)
    if removed4==0:    
        road4_average=road4_acummulator/counter_scanning
    else:
        road4_average=road4_acummulator/(counter_scanning-1)

    whitemarkings_average=whitemarkings_acummulator/counter_scanning


    return road1_average,road2_average,road3_average,road4_average,whitemarkings_average,removed3,removed4
