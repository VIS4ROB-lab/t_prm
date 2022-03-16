cmake_minimum_required(VERSION 3.11)
project(benchmarking)

find_package(ompl REQUIRED)

macro(custom_exec file)
    add_executable(${file} study/${file}.cpp)
    target_link_libraries(${file} benchmarking)
    target_include_directories(${file} PUBLIC include ${OMPL_INCLUDE_DIR})
endmacro()

set(CMAKE_CXX_STANDARD 17)

file(GLOB LIBRARY_SOURCES "src/*.cpp")

add_library(benchmarking ${LIBRARY_SOURCES})
target_link_libraries(benchmarking ${OMPL_LIBRARIES} Eigen3::Eigen TPRM)
target_include_directories(benchmarking PUBLIC include ${OMPL_INCLUDE_DIRS} Eigen3::Eigen TPRM)

custom_exec(study_static_obstacles)
custom_exec(study_dynamic_obstacles)
custom_exec(narrow_gap)