# Steps to build and run the LED control files on Raspberry Pi 5:
# 1. Install CMake and required libraries:
#    sudo apt-get update
#    sudo apt-get install cmake
#    sudo apt-get install libraspberrypi-dev
#    sudo apt-get install libws2811
# 2. Navigate to the project directory:
#    cd /home/hecke/Code/GitRepositories/Sunspot/display
# 3. Create a build directory and navigate into it:
#    mkdir build && cd build
# 4. Run CMake to configure the project:
#    cmake ..
# 5. Build the project:
#    make
# 6. Run the executables:
#    sudo ./led
#    sudo ./led2

cmake_minimum_required(VERSION 3.10)

# Set the project name
project(SunspotDisplay)

# Find FastLED package
find_package(Threads REQUIRED)
# Add these lines after your project() declaration
set(WS2811_INCLUDE_DIR "/usr/local/include/ws2811")
set(WS2811_LIBRARY_DIR "/usr/local/lib")
set(FASTLED_INCLUDE_DIR "/usr/local/include/FastLED")

# Define the source files
set(SRC_LED ${CMAKE_CURRENT_SOURCE_DIR}/src/led.cpp)
#set(SRC_LED2 ${CMAKE_CURRENT_SOURCE_DIR}/src/led2.cpp)

# Add the executable for LED control
add_executable(led ${SRC_LED})
target_include_directories(led PRIVATE ${WS2811_INCLUDE_DIR})
target_link_directories(led PRIVATE ${WS2811_LIBRARY_DIR})
target_link_libraries(led ws2811)

# Add the executable for LED2 control
#add_executable(led2 ${SRC_LED2})
#target_include_directories(led2 PRIVATE ${FASTLED_INCLUDE_DIR})
#target_link_libraries(led2 ${CMAKE_THREAD_LIBS_INIT} FastLED ws2811 wiringPi rt)

# Define the clean rule
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "led")
