"""
This file contains helper functions for things like calculating the
distance to bounding boxes.
"""

import numpy as np

# TODO don't assume uniform frame rate - could record time
def calc_speed(state_for_object, TO_USE=5, verbose=False):
    """
    Right now this function returns the average distance change per frame
     from the last TO_USE  frames.
    If there is no history, aka this is the first frame, it returns None so
     that it will be ignored.
    Also, average with the previous calculated speed, if it exists.

    Arguments:
      state_for_object: list <dict <string: float> >
        The list of states (history) identified by an object.
        Each state in the list is a dictionary of keys like 'distance_y' to
        their appropriate values.
      TO_USE:
        The number of frames to use when calculating speed.
      verbose: boolean
        Whether or not to log extra information.

    Returns: A tuple (Sy, Sx)
      Sy: The relative (to ego) longitudinal speed for this object. Meters/Second
      Sx: The relative (to ego) lateral speed for this object. Meters/Second
    """
    if (len(state_for_object)) <= 1:
        if verbose:
            print("Speed")
            print("state not long enough")
        return None, None
    to_consider = state_for_object[-TO_USE:]
    Dx, nx, Dy, ny = 0.0, 0, 0.0, 0
    for i in range(len(to_consider) - 1):
        if "distance_y" in to_consider[i+1] and "distance_y" in to_consider[i]:
            Dy += (to_consider[i+1]['distance_y']
                   - to_consider[i]['distance_y']
                  )
            ny += 1
        if "distance_x" in to_consider[i+1] and "distance_x" in to_consider[i]:
            Dx += (to_consider[i+1]['distance_x']
                   - to_consider[i]['distance_x']
                  )
            nx += 1
    Sy, Sx = None, None
    if ny > 0:
        Sy = Dy / ny
    if nx > 0:
        Sx = Dx / nx
    if verbose:
        print("Speed")
        print("dy: {}, ny: {}".format(Dy, ny))
        print("dx: {}, nx: {}".format(Dx, nx))
        print("Sy: {}, Sx: {}".format(Sy, Sx))
    return (Sy, Sx)



def triangle_similarity_distance(box, F, W):
    """
    Performs the triangle similarity distance calculation.

    Arguments:
      box: (int, int, int, int)
        (left, right, top, bottom) coordinates of a box in the image.
      F: int
        The focal length of the camera

    Returns:
      The distance to the object in the real world, in meters.
    """
    (left, right, top, bot) = box
    object_width_pixels = right - left
    if object_width_pixels == 0: return np.Inf
    return (W * F) / object_width_pixels

def bottom_bounding_box_distance(
        box,
        im_h,
        im_w,
        rel_horizon=0.5,
        camera_min_angle=25.0,
        camera_height=1.0,
        camera_beta_max=90.0,
        carW=1.8,
        verbose=False):
    """
    Using our first method, compute the distance to the bottom of the bounding
    box. This leverages the knowledge that a horizontal line in the real world
    is also a horizontal line in the image.
    Assumes: flat ground.

    Arguments:
      box: (int, int, int, int)
        (left, right, top, bottom) coordinates of a box in the image.
      im_h: int
        The height of the image, in pixels.
      im_w: int
        The width of the image, in pixels.
      rel_horizon: float
        The relative location of the horizon in the image.
        0.5 means the horizon is in the middle of the image.
        0.0 means the horizon is at the bottom of the image.
      camera_min_angle: float
        The angle made with the ground to see the bottom of the image. Degrees.
      camera_height: float
        The height of the camera off the ground, in real life. Meters.
      camera_beta_max: float
        The maximum angle of the camera to see the furthest horizontal point in
        in the image. Degrees.
      carW: float
        The object width, in meters
      verbose: boolean
        Whether or not to log extra information.

    Return: (dy, dx)
        dy: float
          Longitudinal distance (along centerline) to object. Meters.
        dx: float
          Lateral distance (perpendicular to centerline) to object. Meters.
    """
    horizon_p = rel_horizon * im_h
    if horizon_p == 0: return (0, 0)
    (left, right, top, bot) = box
    d_image = im_h - bot # distance from bottom of image
    if d_image > horizon_p:
        if verbose: print("this box is floating. ignoring. check horizon")
        return None
    phi = ((horizon_p - d_image) / horizon_p) * (90.0 - camera_min_angle)
    dy = camera_height / np.tan(np.deg2rad(phi))
    dx = triangle_for_x(box, im_w, dy, beta_max=camera_beta_max, carW=carW)
    if verbose:
        print("bottom bounding box, dy:", dy, "dx:", dx)
    return (dy, dx)

# to get dx
def triangle_for_x(
        box,
        im_w,
        dy,
        beta_max=90.0,
        carW=1.8,
        verbose=False):
    """
    Finds the lateral distance to the object within the `box`, in the real world.
    Uses the triangle distance method to find this distance.
    Finds distance (lateral) to furthest edge of the box, then subtract half a car width
    to get the distance to the center of the car.
    Must have the longitudinal distance to the object.

    Arguments:
      box: (int, int, int, int)
        (left, right, top, bottom) coordinates of a box in the image.
      im_w: int
        The width of the image, in pixels.
      dy: float
        The longitudinal distance (in the real world) to this object (meters).
      beta_max: float
        The maximum angle of the camera to see the furthest horizontal point in
        in the image. Degrees.
      carW: float
        The object width, in meters
      verbose: boolean
        Whether or not to log extra information.

    Returns:
      dx: the lateral distance to the center of the object, in the real world.
    """
    center_image = im_w / 2.0
    distance_to_far_box_edge = get_distance_far_box_edge(box, im_w)
    beta = (distance_to_far_box_edge / center_image) * (beta_max / 2.0)
    # divide beta_max by two because angle from center is half beta_max
    if verbose:
        print("triangle for x")
        print("left, right, top, bot:", box)
        print("distance far box edge:", distance_to_far_box_edge)
        print("image:", center_image, im_w)
    dx = dy * np.tan(np.deg2rad(beta)) - carW / 2.0
    return dx

def bottom_bounding_box_distance2(
        box,
        im_h,
        im_w,
        camera_focal_len=1000,
        camera_height=1.0,
        carW=1.8,
        verbose=False,
        millimeters_per_pixel=0.285):
    """
    Arguments:
      box: (int, int, int, int)
        (left, right, top, bottom) coordinates of a box in the image.
      im_h: int
        The height of the image, in pixels.
      im_w: int
        The width of the image, in pixels.
      camera_focal_len: int
        The focal length of the camera
      camera_height: float
        The height of the camera off the ground, in real life. Meters.
      carW: float
        The object width, in meters
      verbose: boolean
        Whether or not to log extra information.
      millimeters_per_pixel: float
        The number of millimeters per pixel...

    Return: (dy, dx)
        dy: float
          Longitudinal distance (along centerline) to object. Meters.
        dx: float
          Lateral distance (perpendicular to centerline) to object. Meters.
    """
    (left, right, top, bot) = box
    d_image = im_h - bot # distance from bottom of image
    if d_image == 0 or camera_focal_len == 0: return (0, 0)
    d = bot * millimeters_per_pixel
    d += 1e-10                      # so no division by 0
    dy = (camera_height * camera_focal_len) / d
    distance_to_far_box_edge = get_distance_far_box_edge(box, im_w)
    dx = (dy * distance_to_far_box_edge) / camera_focal_len
    dx -= carW / 2

    if verbose:
        print("bot_box2: dy:", dy, "dx:", dx)
    return (dy, dx)



# 3/4" wide, 2.5" tall, 10 inches away, Focal of ~1000 for built in webcam
def calibrate(
        box,
        im_h,
        im_w,
        object_width=0.18,
        object_height=0.1325,
        distance=0.6096):
    """
    Prints out focal lengths for the camera (computes both width and height to
    make sure they are similar)
    """
    (left, right, top, bot) = box
    object_width_pixels = right - left
    print(object_width_pixels)
    print("F_w= " + str((distance * object_width_pixels) / object_width))
    object_height_pixels = bot-top
    print("F_h= " + str((distance * object_height_pixels) / object_height))

def calibrate2(box, im_h, im_w, height=1.3462):
    """
    Another calibrate method, to find the minimum alpha angle, where alpha
    is the min angle for the camera (used in distance calcs)
    """
    min_d = 1.8288
    alpha = np.rad2deg(np.arctan(min_d/height))
    print("Alpha for min_d: ", str(alpha))
    print("Min observable distance: ", str(min_d))

def get_distance_far_box_edge(box, im_w):
    """
    Simple method to get the distance (In Pixels!) to the far box edge of a
    given box for a given image width.
    """
    (left, right, top, bot) = box
    center_image = im_w / 2
    return max(abs(left - center_image), abs(right - center_image))

def left_of_center(box, im_w):
    """
    return -1 if left of center, 1 otherwise.
    """
    (left, right, top, bot) = box
    center_image = im_w / 2
    if abs(left - center_image) > abs(right - center_image):
        return -1
    return 1
