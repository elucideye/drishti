# This file generated automatically by:
#   generate_sugar_files.py
# see wiki for more info:
#   https://github.com/ruslo/sugar/wiki/Collecting-sources

if(DEFINED SUGAR_OPENCV_CONTRIB_CMAKE_)
  return()
else()
  set(SUGAR_OPENCV_CONTRIB_CMAKE_ 1)
endif()

include(sugar_files)
include(sugar_include)

sugar_files(
  BOOST_PBA_SRCS
  pba/portable_binary_archive.hpp
  pba/portable_binary_iarchive.cpp
  pba/portable_binary_iarchive.hpp
  pba/portable_binary_oarchive.cpp
  pba/portable_binary_oarchive.hpp
  )

