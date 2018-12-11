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
2. python fooWrapper.py
3. should print "Hello" in the terminal
