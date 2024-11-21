cmake_minimum_required(VERSION 3.12)
project(SunspotTest)

# Find OpenCV Package
set(OpenCV_DIR "C:\\Users\\thecr\\Documents\\Zach's Stuff\\opencv\\build")
find_package(OpenCV REQUIRED)

# Include directories from OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

# Create an executable
add_executable(SunspotTest main.cpp)

# Link OpenCV libraries
target_link_libraries(SunspotTest ${OpenCV_LIBS})