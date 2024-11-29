# Steps to build and run the files on Raspberry Pi 5:
# 1. Install CMake and OpenCV:
#    sudo apt-get update
#    sudo apt-get install cmake libopencv-dev
# 2. Navigate to the project directory:
#    cd /home/hecke/Code/GitRepositories/Sunspot/vision
# 3. Create a build directory and navigate into it:
#    mkdir build && cd build
# 4. Run CMake to configure the project:
#    cmake ..
# 5. Build the project:
#    make
# 6. Run the executables:
#    ./detection
#    ./saving
#    ./zachvision
#    ./preloadedmotion

cmake_minimum_required(VERSION 3.10)

# Set the project name
project(SunspotVision)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Include OpenCV headers
include_directories(${OpenCV_INCLUDE_DIRS})

# Define the source files
set(SRC_DETECTION ${CMAKE_CURRENT_SOURCE_DIR}/src/detectionexample.cpp)
set(SRC_SAVING ${CMAKE_CURRENT_SOURCE_DIR}/src/savingexample.cpp)
set(SRC_ZACHVISION ${CMAKE_CURRENT_SOURCE_DIR}/src/zachvision.cpp)
set(SRC_PRELOADEDMOTION ${CMAKE_CURRENT_SOURCE_DIR}/src/preloadedmotion.cpp)

# Add the executables
add_executable(detection ${SRC_DETECTION})
target_link_libraries(detection ${OpenCV_LIBRARIES})

add_executable(saving ${SRC_SAVING})
target_link_libraries(saving ${OpenCV_LIBRARIES} udev)

add_executable(zachvision ${SRC_ZACHVISION})
target_link_libraries(zachvision ${OpenCV_LIBRARIES})

add_executable(preloadedmotion ${SRC_PRELOADEDMOTION})
target_link_libraries(preloadedmotion ${OpenCV_LIBRARIES})

# Define the clean rule
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "detection;saving;zachvision;preloadedmotion")