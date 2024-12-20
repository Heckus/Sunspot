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
set(SRC_mcserial ${CMAKE_CURRENT_SOURCE_DIR}/src/mcserial.cpp)
set(SRC_test ${CMAKE_CURRENT_SOURCE_DIR}/src/test.cpp)

# Add the executable for mcserial
add_executable(mcserial ${SRC_mcserial})
target_link_libraries(mcserial PRIVATE ${WIRINGPI_LIB} Threads::Threads)

add_executable(mctest ${SRC_test})
target_link_libraries(mctest ${WIRINGPI_LIB})

# Define the clean rule
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "mcserial")