# This file generated automatically by:
#   generate_sugar_files.py
# see wiki for more info:
#   https://github.com/ruslo/sugar/wiki/Collecting-sources

if(DEFINED LIB_QTPLUS_QGPLUS_SUGAR_CMAKE_)
  return()
else()
  set(LIB_QTPLUS_QTPLUS_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)

sugar_files(
  DRISHTI_QTPLUS_HDRS
   QGLContext.h
   )

sugar_files(
  DRISHTI_QTPLUS_SRCS
  QGLContext.cpp     
)
