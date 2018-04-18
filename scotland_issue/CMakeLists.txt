include_directories(modules)

# Set the executable along with the required source files
add_executable(scotland_issue test.cc)

# Instruct to link against the ariadne library, and the bdd library
target_link_libraries(scotland_issue ariadne bdd)
