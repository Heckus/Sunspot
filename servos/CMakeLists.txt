# Steps to build and run the files on Raspberry Pi 5:
# 1. Install CMake and gpiod:
#    sudo apt-get update
#    sudo apt-get install cmake gpiod
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
#    sudo ./servo2
#    sudo ./servo3

cmake_minimum_required(VERSION 3.18)

# Set the project name
project(SunspotServos)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_library(WIRINGPI_LIB wiringPi REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)
find_package(Threads REQUIRED)

# Define the source files
set(SRC_SERVO ${CMAKE_CURRENT_SOURCE_DIR}/src/servo.cpp)
set(SRC_SERVO_OOP ${CMAKE_CURRENT_SOURCE_DIR}/src/servo_oop.cpp)
set(SRC_SERVO2 ${CMAKE_CURRENT_SOURCE_DIR}/src/servo2.cpp)
set(SRC_SERVO3 ${CMAKE_CURRENT_SOURCE_DIR}/src/servo3.cpp)

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

# Add the executable for servo2
add_executable(servo2 ${SRC_SERVO2})
target_link_libraries(servo2 
    PRIVATE 
    ${GPIOD_LIBRARIES}
    Threads::Threads
)
include_directories(${GPIOD_INCLUDE_DIRS})

# Add the executable for servo3
add_executable(servo3 ${SRC_SERVO3})
target_link_libraries(servo3 
    PRIVATE 
    ${GPIOD_LIBRARIES}
    Threads::Threads
)
include_directories(${GPIOD_INCLUDE_DIRS})

# Define the clean rule
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "servo;servo_oop;servo2;servo3")