include_directories(modules)

# Set the executable along with the required source files
add_executable(hydropower_non_linear_exp test.cc)

# Instruct to link against the ariadne library, and the bdd library
target_link_libraries(hydropower_non_linear_exp ariadne bdd)
