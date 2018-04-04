# Preferable to avoid warnings during execution
cmake_minimum_required (VERSION 2.8.12)

set(PROJECT"hanoi")

project(${PROJECT} CXX)

include_directories(modules)

# Set the executable along with the required source files
add_executable(hanoi test.cc)

# Instruct to link against the ariadne library, and the bdd library
target_link_libraries(hanoi ariadne bdd)
