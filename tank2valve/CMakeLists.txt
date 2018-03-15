cmake_minimum_required(VERSION 3.5.1)

add_executable(tank2valve test.cc)

target_link_libraries(tank2valve ariadne bdd)