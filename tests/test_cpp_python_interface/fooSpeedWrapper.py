import numpy as np
import cv2

import ctypes
from ctypes import cdll
lib = cdll.LoadLibrary('./libspeedfoo.so')

class Foo(object):
    def __init__(self):
        self.obj = lib.Speed_estimator_new()

    def bar(self):
        lib.Speed_estimator_hello_world(self.obj)

    def image(self):
        image = cv2.imread("../GH_frames/2700.jpg")
        #image = image[10:15,11:15]
        H, W, C = image.shape
        print(H,W,C)
        print(image[0,0,:])
        lib.Speed_estimator_update(self.obj,
        ctypes.c_void_p(image.ctypes.data),
        ctypes.c_int(H),
        ctypes.c_int(W),
        ctypes.c_double(0.0))

    def test_all_images(self):
        time = 0.0
        fps = 30.0
        for im_num in range(2700,2801):
            image = cv2.imread("../GH_frames/"+str(im_num)+".jpg")
            #image = image[10:15,11:15]
            H, W, C = image.shape
            print(H,W,C)
            print(image[0,0,:])
            lib.Speed_estimator_update(self.obj,
                ctypes.c_void_p(image.ctypes.data),
                ctypes.c_int(H),
                ctypes.c_int(W),
                ctypes.c_double(time))
            time += 1.0/fps

f = Foo()
f.bar() #and you will see "Hello" on the screen
#f.image()
f.test_all_images()
