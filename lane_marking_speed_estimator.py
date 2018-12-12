import cv2

import ctypes
from ctypes import cdll
# TODO args
lib = cdll.LoadLibrary('/home/derek/driver_risk_prediction_mono_video/lib_speed_estimator.so')

from driver_risk_utils import general_utils

"""
Make sure the library is compiled beforehand.
See tests/test_cpp_python_interface/README.txt, step 7, but with legitimate names:
g++ -std=c++11 -c -fPIC `pkg-config opencv --cflags` \
    speed_estimator_utils/speed_estimator.cpp \
    -o speed_estimator.o `pkg-config opencv --libs`
g++ -std=c++11 -shared -Wl,-soname,lib_speed_estimator.so \
    -o lib_speed_estimator.so speed_estimator.o `pkg-config opencv --libs`
"""

class LaneMarkingSpeedEstimator(object):
    def __init__(self, display=False):
        self.obj = lib.Speed_estimator_new(ctypes.c_bool(display))
        lib.Speed_estimator_get_speed.restype = ctypes.c_double

    def handle_image(self, image, frame_time):
        """
        Image should be a numpyarray of dtype=uint8.
            This is what is default returned by cv2.imread(), so should be fine.
        frame_time should be a positve float, representing the number of seconds,
            for when the image was captured
        """
        H, W, C = image.shape
        print(H,W,C)
        lib.Speed_estimator_update(self.obj,
            ctypes.c_void_p(image.ctypes.data),
            ctypes.c_int(H),
            ctypes.c_int(W),
            ctypes.c_double(frame_time))

    def get_speed(self):
        """
        must return ego vehicle speed in meters/second
        """
        return general_utils.mph_to_mps(lib.Speed_estimator_get_speed(self.obj))
