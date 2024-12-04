cmake_minimum_required(VERSION 3.10)

# Set the project name
project(SunspotVision)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Find libcamera package
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

# Include OpenCV and libcamera headers
include_directories(${OpenCV_INCLUDE_DIRS} ${LIBCAMERA_INCLUDE_DIRS})

# Define the source files
set(SRC_DETECTION ${CMAKE_CURRENT_SOURCE_DIR}/src/detectionexample.cpp)
set(SRC_SAVING ${CMAKE_CURRENT_SOURCE_DIR}/src/savingexample.cpp)
set(SRC_ZACHVISION ${CMAKE_CURRENT_SOURCE_DIR}/src/zachvision.cpp)
set(SRC_PRELOADEDMOTION ${CMAKE_CURRENT_SOURCE_DIR}/src/preloadedmotion.cpp)
set(SRC_VISION1 ${CMAKE_CURRENT_SOURCE_DIR}/src/vision1.cpp)
set(SRC_VISION2 ${CMAKE_CURRENT_SOURCE_DIR}/src/vision2.cpp)
set(SRC_VISION2_1 ${CMAKE_CURRENT_SOURCE_DIR}/src/vision2_1.cpp)

# Add the executables
add_executable(detection ${SRC_DETECTION})
target_link_libraries(detection ${OpenCV_LIBRARIES})

add_executable(saving ${SRC_SAVING})
target_link_libraries(saving ${OpenCV_LIBRARIES} udev)

add_executable(zachvision ${SRC_ZACHVISION})
target_link_libraries(zachvision ${OpenCV_LIBRARIES})

add_executable(preloadedmotion ${SRC_PRELOADEDMOTION})
target_link_libraries(preloadedmotion ${OpenCV_LIBRARIES})

add_executable(vision1 ${SRC_VISION1})
target_link_libraries(vision1 ${OpenCV_LIBRARIES})

add_executable(vision2 ${SRC_VISION2})
target_link_libraries(vision2 ${OpenCV_LIBRARIES})

add_executable(vision2_1 ${SRC_VISION2_1})
target_link_libraries(vision2_1 ${OpenCV_LIBRARIES})

# Define the clean rule
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "detection;saving;zachvision;preloadedmotion;vision1;vision2;vision2_1")
