# Steps to build and run the files on Raspberry Pi 5:
# 1. Install CMake and WiringPi:
#    sudo apt-get update
#    sudo apt-get install cmake wiringpi
# 2. Navigate to the project directory:
#    cd /home/hecke/Code/GitRepositories/Sunspot/servos
# 3. Create a build directory and navigate into it:
#    mkdir build && cd build
# 4. Run CMake to configure the project:
#    cmake ..
# 5. Build the project:
#    make
# 6. Run the executables:
#    sudo ./servo
#    sudo ./servo_oop

cmake_minimum_required(VERSION 3.18)

# Set the project name
project(SunspotServos)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_library(WIRINGPI_LIB wiringPi REQUIRED)
find_package(Threads REQUIRED)

# Define the source files
set(SRC_SERVO ${CMAKE_CURRENT_SOURCE_DIR}/src/servo.cpp)
set(SRC_SERVO_OOP ${CMAKE_CURRENT_SOURCE_DIR}/src/servo_oop.cpp)

# Add the executable for servo
add_executable(servo ${SRC_SERVO})
target_link_libraries(servo 
    PRIVATE 
    ${WIRINGPI_LIB}
    Threads::Threads
)

# Add the executable for servo_oop
add_executable(servo_oop ${SRC_SERVO_OOP})
target_link_libraries(servo_oop 
    PRIVATE 
    ${WIRINGPI_LIB}
    Threads::Threads
)

# Define the clean rule
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "servo;servo_oop")