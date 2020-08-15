# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Specific
set(TRIPLE arm-linux-gnueabihf)
set(PY_VERSION 38)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_FLAGS -Wno-psabi)
set(CMAKE_CXX_FLAGS -Wno-psabi)

# Generic
function(require_env name)
if("$ENV{${name}}" STREQUAL "")
    message(FATAL_ERROR "Required environment variable ${name} not defined")
endif()
endfunction()

require_env(SYSROOT)
require_env(ROS_WS_INSTALL_PATH)
require_env(ROS_DISTRO)

set(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_VERSION 1)

set(CMAKE_C_COMPILER ${TRIPLE}-gcc)
set(CMAKE_CXX_COMPILER ${TRIPLE}-g++)

set(CMAKE_SYSROOT $ENV{SYSROOT})
set(CMAKE_FIND_ROOT_PATH $ENV{ROS_WS_INSTALL_PATH})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_SYSROOT}/opt/ros/${ROS_DISTRO}/lib)

set(PYTHON_SOABI cpython-${PY_VERSION}-${TRIPLE})
set(THREADS_PTHREAD_ARG 0)
