rm *.out

g++ -std=c++11 -o \
  test_speed_estimator_class_hwy101.out \
  `pkg-config opencv --cflags` \
  test_speed_estimator_class_hwy101.cpp \
  ../speed_estimator_utils/speed_estimation_initialization.cpp \
  ../speed_estimator_utils/speed_estimation_update.cpp \
  ../speed_estimator_utils/speed_estimation_update_image.cpp \
  ../speed_estimator_utils/speed_estimation_whitemark_detection_and_speed_calculation.cpp \
  ../speed_estimator_utils/speed_estimation_whitemark_detection_and_speed_calculation_time.cpp \
  ../speed_estimator_utils/speed_estimation_get_speed.cpp \
  `pkg-config opencv --libs`

time ./test_speed_estimator_class_hwy101.out
