include_directories(modules)

# Set the executable along with the required source files
add_executable(hydropower_linear_diag test.cc)

# Instruct to link against the ariadne library, and the bdd library
target_link_libraries(hydropower_linear_diag ariadne bdd)
