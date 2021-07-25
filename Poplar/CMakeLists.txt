cmake_minimum_required(VERSION 3.5)
project(Poplar)

set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
    message(SEND_ERROR "In-source builds are not allowed.")
endif ()

set(CMAKE_COLOR_MAKEFILE   ON)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 14)
set(BUILD_TYPE_RELEASE TRUE)

set(CMAKE_CXX_FLAGS "-O3 -mavx -mfma -no-pie -march=native -ggdb -fPIC")
set(CMAKE_C_FLAGS "-O3 -fPIC -ggdb -mavx -mfma -march=native -std=gnu99 -I.")

list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots/")
find_package(pinocchio REQUIRED)
if (pinocchio_FOUND)
    # message("pinocchio found")
    include_directories(SYSTEM ${pinocchio_INCLUDE_DIRS})
    # message("pinocchio include dir at " ${pinocchio_INCLUDE_DIRS})
else ()
    message(FATAL_ERROR "Cannot find pinocchio")
endif ()

SET(PoplarLib_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/path.h.cmake
        ${CMAKE_BINARY_DIR}/path_PoplarLib.h)

include_directories(${PoplarLib_PATH}/include)
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PoplarLib_PATH}/third_party/qpOASES/include)
include_directories(${PoplarLib_PATH}/third_party/eiquadprog/include)
include_directories(/usr/include/eigen3)

link_directories(${CMAKE_BINARY_DIR})

add_subdirectory(third_party)
add_subdirectory(test)

file(GLOB_RECURSE source ${PoplarLib_PATH}/src/*.cpp)

add_library(Poplar ${source})
target_link_libraries(Poplar qpOASES eiquadprog pinocchio::pinocchio rt pthread)

