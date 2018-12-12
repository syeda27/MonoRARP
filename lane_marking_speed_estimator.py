import cv2

import ctypes
from ctypes import cdll
lib = cdll.LoadLibrary('./lib_speed_estimator.so')

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
    def __init__(self):
        self.obj = lib.Speed_estimator_new()

    def handle_image(self, image, frame_time):
        """
        Image should be a numpyarray of dtype=uint8.
            This is what is default returned by cv2.imread(), so should be fine.
        frame_time should be a positve float, representing the number of seconds,
            for when the image was captured
        """
        H, W, C = image.shape
        lib.Speed_estimator_update(self.obj,
            ctypes.c_void_p(image.ctypes.data),
            ctypes.c_int(H),
            ctypes.c_int(W),
            ctypes.c_double(frame_time))

    def get_speed(self):
        return lib.Speed_estimator_get_speed(self.obj)
