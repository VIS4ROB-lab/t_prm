cmake_minimum_required(VERSION 3.11)
project(benchmarking)

find_package(OMPL REQUIRED)

macro(custom_exec file)
    add_executable(${file} study/${file}.cpp)
    target_link_libraries(${file} benchmarking)
    target_include_directories(${file} PRIVATE include ${OMPL_INCLUDE_DIR})
endmacro()

set(CMAKE_CXX_STANDARD 17)

file(GLOB LIBRARY_SOURCES "src/*.cpp")

add_library(benchmarking SHARED ${LIBRARY_SOURCES})
target_link_libraries(benchmarking ${OMPL_LIBRARIES} Eigen3::Eigen TPRM)
target_include_directories(benchmarking PUBLIC include ${OMPL_INCLUDE_DIRS} Eigen3::Eigen TPRM)

custom_exec(simple_comparison)
custom_exec(static_obstacles)
custom_exec(dynamic_obstacles)
custom_exec(static_num_queries)
custom_exec(narrow_gap)