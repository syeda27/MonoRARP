'''
This file contains some simple and some not-so-simple functions to help make the
 diplay of for the driver risk estimation system
'''
import numpy as np
import cv2

try:
    from driver_risk_utils import general_utils
except ImportError:  # For testing
    import general_utils

# TODO modularize
def display(args, STATE, RISK_ESTIMATOR, im, boxes, do_convert=True, labels=[], fps=6.0,
        left_margin=12, top_margin=36, space=36,
        calculate_risk=True):
    if type(im) is not np.ndarray:
        imgcv = cv2.imread(im)
    else: 
        imgcv = im
    im_height, im_width, _ = imgcv.shape
    thick = int((im_height + im_width) // 300)
    color = (0,50,255) # BGR
    black = (0, 0 ,0)
    if len(labels) < len(boxes):
        labels.extend([""] * (len(boxes) - len(labels)))
    for i,b in enumerate(boxes):
        if do_convert:
            (left, right, top, bot) = general_utils.convert(im_height, im_width, b)
        else:
            (left, right, top, bot) = b
        aspect_ratio_off = general_utils.check_aspect_ratio(b)
        if labels[i] != "car" or aspect_ratio_off:
            cv2.rectangle(imgcv,
                        (int(left), int(top)), (int(right), int(bot)),
                        (0,0,50), int(thick/3))
            continue
        this_state = STATE.update_state((left, right, top, bot), 
                im_height, im_width, args, object_key=i)
        text = ""
        text2 = ""
        object_label = "obj: " + str(i)
        if this_state is not None:
            if "distance_x" in this_state:
                text += "dx: {0:.2f}, ".format(this_state['distance_x'])
            if "distance_y" in this_state:
                text += "dy: {0:.2f}".format(this_state['distance_y'])
            if 'speed_x' in this_state:
                text2 += "sx: {0:.2f}, ".format(\
                        this_state['speed_x']*fps)
            if 'speed_y' in this_state:
                text2 += "sy: {0:.2f}".format(this_state['speed_y']*fps)
            # state info in top left:
            outline_text(imgcv, object_label, left_margin, 
                    top_margin+space*(3*i), 
                    im_height, black, color, thick)
            outline_text(imgcv, text, left_margin, 
                    top_margin+space*(3*i+1), 
                    im_height, black, color, thick)
            outline_text(imgcv, text2, left_margin, 
                    top_margin+space*(3*i+2),
                    im_height, black, color, thick)
            
        # object id on box:
        outline_text(imgcv, object_label, int(left), int(top), im_height, black, color, thick)
        cv2.rectangle(imgcv,
                        (int(left), int(top)), (int(right), int(bot)),
                        color, int(thick/3))
    if calculate_risk:
        risk = RISK_ESTIMATOR.get_risk(STATE, risk_type="online", n_sims=50, verbose=False)
    else:
        risk = RISK_ESTIMATOR.prev_risk
    outline_text(imgcv, "risk: {0:.2f}".format(risk), 
                int(im_width / 2 - space), 
                im_height - space, im_height, 
                black, color, thick)
    outline_text(imgcv, "ego speed: {0:.2f} mph".format(STATE.get_ego_speed_mph()), 
                int(im_width / 2 - space), 
                im_height - space - space, im_height, 
                black, color, thick)
    return imgcv

def outline_text(imgcv, text, left, top, imh, color1, color2, thick):
    cv2.putText(imgcv, text,
            (left, top-12),  # TODO make these values parameters 
            0, 1e-3*imh, color1, int(2*thick/3))
    cv2.putText(imgcv, text,
            (left, top-12), 
            0, 1e-3*imh, color2, int(1*thick/4))

