cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)

set(PROJECT"ariadneExample")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

project(${PROJECT} CXX)

add_subdirectory(hydropower/linear/diagonal)
add_subdirectory(hydropower/linear/exponential)
add_subdirectory(scotland_issue)
add_subdirectory(crazy_river/hysteresis)
add_subdirectory(lock_system)
