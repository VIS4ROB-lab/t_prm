cmake_minimum_required(VERSION 3.11)

project(TPRM_Examples)
set(CMAKE_BUILD_TYPE Release)

add_executable(static_avoidance static_avoidance.cpp)
add_executable(dynamic_avoidance dynamic_avoidance.cpp)

target_link_libraries(static_avoidance TPRM)
target_link_libraries(dynamic_avoidance TPRM)
