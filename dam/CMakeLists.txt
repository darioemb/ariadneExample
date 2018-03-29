# Preferable to avoid warnings during execution
cmake_minimum_required (VERSION 2.8.12)

set(PROJECT"dam")

project(${PROJECT} CXX)

include_directories(modules)

# Set the executable along with the required source files
add_executable(dam test.cc)

# Instruct to link against the ariadne library, and the bdd library
target_link_libraries(dam ariadne bdd)
