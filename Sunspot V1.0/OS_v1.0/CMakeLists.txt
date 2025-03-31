cmake_minimum_required(VERSION 3.10)

# Set the project name
project(OS_v1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the executable - include FrameQueue.cpp if it exists
add_executable(OS_v1.0 
    src/main.cpp 
    include/OS_tools.cpp 
    include/INA219.cpp
    include/FrameQueue.cpp  # Add this line
)

# Find OpenCV package
find_package(OpenCV REQUIRED)
target_include_directories(OS_v1.0 PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(OS_v1.0 PRIVATE ${OpenCV_LIBS})

# Find Threads package
find_package(Threads REQUIRED)
target_link_libraries(OS_v1.0 PRIVATE Threads::Threads)

# Link wiringPi library
find_library(WIRINGPI_LIB wiringPi)
if(WIRINGPI_LIB)
    target_link_libraries(OS_v1.0 PRIVATE ${WIRINGPI_LIB})
else()
    message(FATAL_ERROR "wiringPi library not found")
endif()

# Find GTKMM package
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)
target_include_directories(OS_v1.0 PRIVATE ${GTKMM_INCLUDE_DIRS})
target_link_libraries(OS_v1.0 PRIVATE ${GTKMM_LIBRARIES})

# Include directories
target_include_directories(OS_v1.0 PRIVATE ${CMAKE_SOURCE_DIR}/include)
