g++ -std=c++11 -c -fPIC `pkg-config opencv --cflags` \
    ../speed_estimator_utils/speed_estimator.cpp \
    -o speed_estimator.o `pkg-config opencv --libs`
g++ -std=c++11 -shared -Wl,-soname,lib_speed_estimator.so \
    -o ../lib_speed_estimator.so speed_estimator.o `pkg-config opencv --libs`
