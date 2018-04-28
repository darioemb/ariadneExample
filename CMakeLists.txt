cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)

set(PROJECT"ariadneExample")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

project(${PROJECT} CXX)

add_subdirectory(hydropower)
add_subdirectory(scotland_issue)
add_subdirectory(crazy_river/hard)
add_subdirectory(crazy_river/easy)
add_subdirectory(lock_system/hard)
add_subdirectory(lock_system/easy)
