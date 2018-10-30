cmake_minimum_required(VERSION 3.1)

set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-pragma-once-outside-header")

project (DriverRiskPrediction)
set(CMAKE_BINARY_DIR ..)
set(CMAKE_SOURCE_DIR ${CMAKE_BINARY_DIR}/Compile)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include_directories("${CMAKE_BINARY_DIR}")


add_executable(Test ${CMAKE_BINARY_DIR}/src/main_test.cpp)
add_library(Scene ${CMAKE_BINARY_DIR}/src/scene.cpp)
add_library(Vehicle ${CMAKE_BINARY_DIR}/src/vehicle.cpp)
add_library(State ${CMAKE_BINARY_DIR}/src/obj_det_state.cpp)
add_library(Risk ${CMAKE_BINARY_DIR}/src/risk_predictor.cpp)
add_library(Types ${CMAKE_BINARY_DIR}/src/types.cpp)
add_library(VehicleState ${CMAKE_BINARY_DIR}/src/vehicle_state.cpp)

target_link_libraries (Test Scene)
target_link_libraries (Test Vehicle)
target_link_libraries (Test State)
target_link_libraries (Test Risk)
target_link_libraries (Test Types)
target_link_libraries (Test VehicleState)

# add the install targets
install (TARGETS Test DESTINATION bin)
