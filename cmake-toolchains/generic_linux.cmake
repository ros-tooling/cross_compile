# Copyright (c) 2018, ARM Limited.
# SPDX-License-Identifier: Apache-2.0

if("$ENV{TARGET_ARCH}" STREQUAL "" OR
   "$ENV{CROSS_COMPILE}" STREQUAL "" OR
   "$ENV{SYSROOT}" STREQUAL "" OR
   "$ENV{ROS2_INSTALL_PATH}" STREQUAL "" OR
   "$ENV{PYTHON_SOABI}" STREQUAL "")
    message(FATAL_ERROR "Target environment variables not defined")
endif()

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR $ENV{TARGET_ARCH})

# Specify the cross compiler
set(CMAKE_C_COMPILER $ENV{CROSS_COMPILE}gcc)
set(CMAKE_CXX_COMPILER $ENV{CROSS_COMPILE}g++)

# Specify the target file system
set(CMAKE_SYSROOT $ENV{SYSROOT})

set(CMAKE_FIND_ROOT_PATH $ENV{ROS2_INSTALL_PATH})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Specify the python SOABI
set(PYTHON_SOABI $ENV{PYTHON_SOABI})

if(NOT TARGET Qt5::moc)
  set(QT_MOC_EXECUTABLE /usr/bin/moc)
  add_executable(Qt5::moc IMPORTED)
  set_property(TARGET Qt5::moc PROPERTY IMPORTED_LOCATION ${QT_MOC_EXECUTABLE})
endif()

# This assumes that pthread will be available on the target system
# (this emulates that the return of the TRY_RUN is a return code "0")
set(THREADS_PTHREAD_ARG "0"
  CACHE STRING "Result from TRY_RUN" FORCE)
