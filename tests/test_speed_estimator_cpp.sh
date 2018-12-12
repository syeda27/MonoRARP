# default
rm *.out
g++ -std=c++11 -o \
  test_speed_estimator_class_hwy101.out \
  `pkg-config opencv --cflags` \
  test_speed_estimator_class_hwy101.cpp \
  ../speed_estimator_utils/speed_estimation_initialization.cpp \
  ../speed_estimator_utils/speed_estimation_update_image.cpp \
  ../speed_estimator_utils/speed_estimation_whitemark_detection_and_speed_calculation_time.cpp \
  ../speed_estimator_utils/speed_estimation_get_speed.cpp \
  `pkg-config opencv --libs`

time ./test_speed_estimator_class_hwy101.out >> testhwy101.log

# our "new" version
g++ -std=c++11 -o \
  test_speed_estimator_class_hwy1012v2.out \
  `pkg-config opencv --cflags` \
  test_speed_estimator_class_hwy101.cpp \
  ../speed_estimator_utils/speed_estimator.cpp \
  `pkg-config opencv --libs`
time ./test_speed_estimator_class_hwy101.out >> testhwy101v2.log


# on the other video
g++ -std=c++11 -o \
  test_speed_estimator_class_11a.out \
  `pkg-config opencv --cflags` \
  test_speed_estimator_class_11a.cpp \
  ../speed_estimator_utils/speed_estimator.cpp \
  `pkg-config opencv --libs`

time ./test_speed_estimator_class_11a.out >> test11a.log
