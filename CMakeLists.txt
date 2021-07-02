cmake_minimum_required(VERSION 3.14)
project(PoplarDigit)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)
set(CMAKE_COLOR_MAKEFILE ON)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(BUILD_TYPE_RELEASE TRUE)
SET(THIS_COM "${PROJECT_SOURCE_DIR}")
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_BINARY_DIR}/Configuration.h)
set(CMAKE_CXX_FLAGS "-O3 -mavx -mfma -march=native -no-pie -fPIC")

list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots/")
find_package(pinocchio REQUIRED)
if(pinocchio_FOUND)
    include_directories(SYSTEM ${pinocchio_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "Cannot find pinocchio")
endif()

include_directories(${THIS_COM}/TypeDefinition/include)
include_directories(${CMAKE_BINARY_DIR})
include_directories(/usr/include/eigen3)
include_directories(SYSTEM ${THIS_COM}/third_party/qpOASES/include)
include_directories(SYSTEM ${THIS_COM}/third_party/eiquadprog/include)
include_directories(SYSTEM ${THIS_COM}/third_party/Task-Space-Control/include)
include_directories(${THIS_COM}/third_party/RobotWrapper/include)

link_directories(${CMAKE_BINARY_DIR})

link_directories(${CMAKE_BINARY_DIR})
link_directories(${THIS_COM}/Simulation/lib/)

add_subdirectory(third_party)
add_subdirectory(Simulation)
add_subdirectory(User)

