cmake_minimum_required(VERSION 3.11)

project(TPRM)

include(FetchContent)
macro(fetch what)
    FetchContent_GetProperties("${what}")
    if(NOT ${${what}_POPULATED})
        message(STATUS "fetching ${what} ...")
        FetchContent_Populate(${what})
    endif()
    mark_as_advanced(${${what}_SOURCE_DIR})
endmacro()

set(CMAKE_CXX_STANDARD 17)

find_package (Eigen3 3.3 NO_MODULE)
if(NOT TARGET Eigen3::Eigen)
  # Remove entry that has not been found
  unset(Eigen3_DIR CACHE)

  # Now download eigen3
  # download as zip, so it's a bit smaller than downloading the whole repo
  FetchContent_Declare(
      eigen #
      URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip #
      URL_HASH MD5=994092410ba29875184f7725e0371596 #
  )
  fetch(eigen)
  add_library(eigen INTERFACE)
  add_library (Eigen3::Eigen ALIAS eigen)
  target_include_directories(eigen INTERFACE ${eigen_SOURCE_DIR})
endif(NOT TARGET Eigen3::Eigen)

find_package(OpenMP REQUIRED)

file(GLOB TPRM_SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB TPRM_HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)

add_library(${PROJECT_NAME} SHARED ${TPRM_SRC_FILES} ${TPRM_HEADER_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC "include") # TODO: build/install scopes

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen OpenMP::OpenMP_CXX)


add_subdirectory(examples)

# TODO: INSTALL