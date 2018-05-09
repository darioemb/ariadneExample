

include_directories(modules)

# Set the executable along with the required source files
add_executable(lock_system  test.cc)

# Instruct to link against the ariadne library, and the bdd library
target_link_libraries(lock_system ariadne bdd)
