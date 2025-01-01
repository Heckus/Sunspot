cmake_minimum_required(VERSION 3.10)

# Set the project name
project(OS_v1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the executable
add_executable(OS_v1.0 src/main.cpp include/OS_tools.cpp include/INA219.cpp)

# Find OpenCV package
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(OS_v1.0 ${OpenCV_LIBS})

# Find Threads package
find_package(Threads REQUIRED)
target_link_libraries(OS_v1.0 Threads::Threads)

# Link wiringPi library
target_link_libraries(OS_v1.0 wiringPi)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
