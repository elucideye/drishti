# This file generated automatically by:
#   generate_sugar_files.py
# see wiki for more info:
#   https://github.com/ruslo/sugar/wiki/Collecting-sources

if(DEFINED LIB_TESTLIB_TESTLIB_SUGAR_CMAKE_)
  return()
else()
  set(LIB_TESTLIB_TESTLIB_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)

sugar_files(
  DRISHTI_TESTLIB_HDRS
  drishti_testlib.h
  drishti_test_utils.h
   )

sugar_files(
  DRISHTI_TESTLIB_SRCS
  drishti_test_main.cpp
  )

