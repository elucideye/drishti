# @file   append.cmake
# @brief  CMake helper to append libs to master list
#
# \copyright Copyright 2014-2018 Elucideye, Inc. All rights reserved.
# \license{This project is released under the 3 Clause BSD License.}
#
# This is a helper routine for drishti_merge_libraries.cmake

cmake_minimum_required(VERSION 3.1)

if("${LIBRARIES_LIST}" STREQUAL "")
  message(FATAL_ERROR "LIBRARIES_LIST is empty")
endif()

if("${LIBRARY}" STREQUAL "")
  message(FATAL_ERROR "LIBRARY is empty")
endif()

if(NOT EXISTS "${LIBRARY}")
  message(FATAL_ERROR "Library file not found: ${LIBRARY}")
endif()

file(APPEND "${LIBRARIES_LIST}" "${LIBRARY}\n")
