#
# Copyright (c) 2019-2020 CNRS
#
# This file is part of eiquadprog.
#
# eiquadprog is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# eiquadprog is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with eiquadprog.  If not, see <https://www.gnu.org/licenses/>.
#

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
project(eiquadprog)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)

set(CMAKE_CXX_FLAGS "-O3 -mavx -mfma -march=native -no-pie -ggdb")

include_directories(./include)

include_directories(/usr/include/eigen3)

file(GLOB_RECURSE source "./src/*.cpp" "./include/*.hpp")

ADD_LIBRARY(eiquadprog SHARED ${source})
