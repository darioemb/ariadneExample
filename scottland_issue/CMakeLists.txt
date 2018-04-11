# Preferable to avoid warnings during execution
cmake_minimum_required (VERSION 2.8.12)

set(PROJECT"scottland_issue")


project(${PROJECT} CXX)

include_directories(modules)

# Set the executable along with the required source files
add_executable(scottland_issue test.cc)

# Instruct to link against the ariadne library, and the bdd library
target_link_libraries(scottland_issue ariadne bdd)
