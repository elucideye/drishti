# This file generated automatically by:
#   generate_sugar_files.py
# see wiki for more info:
#   https://github.com/ruslo/sugar/wiki/Collecting-sources

if(DEFINED LIB_GEOMETRY_GEOMETRY_SUGAR_CMAKE_)
  return()
else()
  set(LIB_GEOMETRY_GEOMETRY_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)

sugar_files(DRISHTI_GEOMETRY_SRCS
  Ellipse.cpp
  Primitives.cpp
  Rectangle.cpp
  motion.cpp
  )

sugar_files(DRISHTI_GEOMETRY_HDRS_PUBLIC
  Ellipse.h
  Primitives.h
  Rectangle.h
  drishti_geometry.h
  motion.h
  )
