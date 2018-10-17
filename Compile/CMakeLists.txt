cmake_minimum_required(VERSION 3.1)

set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -Wall")

project (DriverRiskPrediction)
set(CMAKE_BINARY_DIR ..)
set(CMAKE_SOURCE_DIR ${CMAKE_BINARY_DIR}/Compile)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include_directories("${CMAKE_BINARY_DIR}")


add_executable(Test ${CMAKE_BINARY_DIR}/src/scene.cpp)
add_library(Vehicle ${CMAKE_BINARY_DIR}/src/vehicle.cpp)
target_link_libraries (Test Vehicle)

# add the install targets
install (TARGETS Test DESTINATION bin)
