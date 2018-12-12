general idea:
https://stackoverflow.com/questions/145270/calling-c-c-from-python

for mac:
https://stackoverflow.com/questions/496664/c-dynamic-shared-library-on-linux

run tests with:
1. compile the .so through the instructions in the links above, for the cpp file foo.cpp
  - on Mac I ran: `g++ -dynamiclib -flat_namespace foo.cpp -o libfoo.so`
  - on Ubuntu:
    - `g++ -c -fPIC foo.cpp -o foo.o`
    - `g++ -shared -Wl,-soname,libfoo.so -o libfoo.so foo.o`
2. `python fooWrapper.py`
3. should print "Hello" in the terminal
4. compiling foo, openCV version:
  - on Ubuntu:
    - ```
    g++ -std=c++11 -c -fPIC `pkg-config opencv --cflags` foo_opencv.cpp -o foo.o `pkg-config opencv --libs`
    ```
    - ```
    g++ -std=c++11 -shared -Wl,-soname,libfoo.so -o libfoo.so foo.o `pkg-config opencv --libs`
    ```
5. `python fooWrapper.py`
6. should print "Hello" and run the images for the speed test.
7. Compile the speed estimator version
  - on Ubuntu:
    - ```
    g++ -std=c++11 -c -fPIC `pkg-config opencv --cflags` ../../speed_estimator_utils/speed_estimator.cpp -o foospeed.o `pkg-config opencv --libs`
    g++ -std=c++11 -shared -Wl,-soname,libspeedfoo.so -o libspeedfoo.so foospeed.o `pkg-config opencv --libs`
    ```
8. `python fooSpeedWrapper.py`
