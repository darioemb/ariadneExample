include_directories(modules)

# Set the executable along with the required source files
add_executable(crazy_river_easy test.cc)

# Instruct to link against the ariadne library, and the bdd library
target_link_libraries(crazy_river_easy ariadne bdd)
