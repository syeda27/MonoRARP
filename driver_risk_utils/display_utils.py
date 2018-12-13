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

"""
Display_args
These values are necessary for displaying the information to the user.
Currently just values, but we obviously want these to be configurable.
"""
class DisplayArgs:
    def __init__(self):
        self.left_margin = 12 # pixels
        self.top_margin = 36
        self.space = 36
        self.text_outline_offset = 12
        self.text_height_mult = 1e-3
        self.color = (0, 50, 255) # BGR
        self.black = (0, 0, 0)
        self.invalid_color = (0, 0, 50)
        self.thickness = 300
        self.thick = None

    def set_im(self, im):
        self.im_height, self.im_width, _ = im.shape
        self.set_thick()

    def set_thick(self):
        self.thick = self.get_thick()

    def get_thick(self):
        if self.thick is not None:
            return self.thick
        return int( int((self.im_height + self.im_width) // self.thickness) / 3)

def make_line(im, p1, p2, color=255, thickness=1, type=cv2.LINE_AA):
    cv2.line(im,
             (int(p1[0]), int(p1[1])),
             (int(p2[0]), int(p2[1])),
             color,
             thickness,
             type)

def make_rectangle(im, b, color, thickness):
    #   (left, right, top, bot) = b
    cv2.rectangle(im,
                  (int(b[0]), int(b[2])),
                  (int(b[1]), int(b[3])),
                  color,
                  thickness)

def outline_rectangle(im, b, disp_args, object_key=None):
    make_rectangle(im, b, disp_args.black, disp_args.get_thick()*2)
    make_rectangle(im, b, disp_args.color, disp_args.get_thick())
    if object_key is not None:
        outline_text(
            im,
            "OBJ: {}".format(object_key),
            int(b[0]),
            int(b[2]),
            disp_args,
            thick_mult=1)

def make_text(obj_key, this_state, frame_time):
    text = ""
    text2 = ""
    object_label = "obj: " + obj_key
    if this_state is not None:
        if "distance_x" in this_state:
            text += "dx: {0:.2f} m, ".format(this_state['distance_x'])
        if "distance_y" in this_state:
            text += "dy: {0:.2f} m".format(this_state['distance_y'])
        if 'speed_x' in this_state:
            text2 += "sx: {0:.2f} mps, ".format(this_state['speed_x'])
        if 'speed_y' in this_state:
            text2 += "sy: {0:.2f} mps".format(this_state['speed_y'])
    return [object_label, text, text2]

def outline_object_text(text_list, imgcv, disp_args, i):
    im_height, im_width, _ = imgcv.shape
    n = len(text_list)
    for text_j in range(n):
        outline_text(imgcv,
                     text_list[text_j],
                     disp_args.left_margin,
                     disp_args.top_margin + disp_args.space * (n * i + text_j),
                     disp_args,
                     thick_mult=1)

def outline_global_text(img, risk, ego_speed_mph, disp_args):
    horiz = int(disp_args.im_width / 2 - disp_args.space)
    vert = disp_args.im_height - disp_args.space
    outline_text(img,
                 "risk: {0:.2f}".format(risk),
                 horiz,
                 vert,
                 disp_args,
                 thick_mult=1)
    outline_text(img,
                 "ego speed: {0:.2f} mph, {1:.2f} mps".format(
                    ego_speed_mph, general_utils.mph_to_mps(ego_speed_mph)),
                 horiz,
                 vert - disp_args.space,
                 disp_args,
                 thick_mult=1)


def outline_text(imgcv, text, left, top, disp_args, thick_mult=1):
    cv2.putText(imgcv,
                text,
                (left, top - disp_args.text_outline_offset),
                0, # fontFace
                disp_args.text_height_mult * disp_args.im_height,
                disp_args.black,
                2*disp_args.get_thick()*thick_mult)
    cv2.putText(imgcv,
                text,
                (left, top - disp_args.text_outline_offset),
                0, # fontFace
                disp_args.text_height_mult * disp_args.im_height,
                disp_args.color,
                disp_args.get_thick()*thick_mult)


def display(args,
            state,
            risk,
            speed,
            im,
            boxes,
            labels=[],
            fps=6.0,
            frame_time=None,
            disp_args=DisplayArgs()):
    """
    We leave this function here, even though display.py exists, in case we ever
    want to display without maintaining a display class object.
    """
    if type(im) is not np.ndarray:
        imgcv = cv2.imread(im)
    else:
        imgcv = im
    disp_args.set_im(imgcv)
    for i,b in enumerate(boxes):
        aspect_ratio_off = general_utils.check_aspect_ratio(b)
        (left, right, top, bot) = b
        if labels[i] != "car" or aspect_ratio_off:
            make_rectangle(imgcv, b, disp_args.invalid_color, disp_args.thick)
            continue

        text = make_text(str(i), state[i], frame_time)
        # object id on box:
        outline_object_text(text, imgcv, disp_args, i)
        outline_rectangle(imgcv, b, disp_args)
    outline_global_text(imgcv, risk, speed, disp_args)
    return imgcv
