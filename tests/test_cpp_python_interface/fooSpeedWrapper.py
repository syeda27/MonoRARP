from ctypes import cdll
lib = cdll.LoadLibrary('./libspeedfoo.so')

class Foo(object):
    def __init__(self):
        self.obj = lib.Speed_estimator_new()

    def bar(self):
        lib.Speed_estimator_hello_world(self.obj)

f = Foo()
f.bar() #and you will see "Hello" on the screen
