cmake_minimum_required(VERSION 3.10)

# Specify the C and C++ compilers

set(CMAKE_C_COMPILER "C:/msys64ucrt64/bin/gcc.exe")
set(CMAKE_CXX_COMPILER "C:msys64\\ucrt64\\bin\\g++.exe")

# Set the project name
project(SunspotVision)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Include OpenCV headers
include_directories(${OpenCV_INCLUDE_DIRS})

# Define the source files
set(SRC_DETECTION ${CMAKE_CURRENT_SOURCE_DIR}/src/detectionexample.cpp)
set(SRC_SAVING ${CMAKE_CURRENT_SOURCE_DIR}/src/savingexample.cpp)

# Add the executable for detection
add_executable(detection ${SRC_DETECTION})
target_link_libraries(detection ${OpenCV_LIBS})

# Add the executable for saving
add_executable(saving ${SRC_SAVING})
target_link_libraries(saving ${OpenCV_LIBS})

# Define the clean rule
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "detection;saving")