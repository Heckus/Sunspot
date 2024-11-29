cmake_minimum_required(VERSION 3.18)

# Set the project name
project(SunspotSerial)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_library(WIRINGPI_LIB wiringPi REQUIRED)
find_package(Threads REQUIRED)

# Define the source file
set(SRC_DIRECTPI ${CMAKE_CURRENT_SOURCE_DIR}/src/directpi.cpp)

# Add the executable for directpi
add_executable(directpi ${SRC_DIRECTPI})
target_link_libraries(directpi 
    PRIVATE 
    ${WIRINGPI_LIB}
    Threads::Threads
)

# Define the clean rule
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "directpi")