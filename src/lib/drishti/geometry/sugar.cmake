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
  DynamicObject.cpp
  Ellipse.cpp
  EllipseSerializer.cpp
  Primitives.cpp
  Rectangle.cpp
  conicCen2Par.cpp
  conicPar2Cen.cpp
  motion.cpp
  )

sugar_files(DRISHTI_GEOMETRY_HDRS_PUBLIC
  ConicSection.h
  Cylinder.h
  DynamicObject.h  
  Ellipse.h
  EllipseSerializer.h
  Mesh3D.h
  Primitives.h
  Rectangle.h
  StaticObject.h
  drishti_geometry.h
  fitEllipse.h
  getPointsOnLine.h
  intersectConicLine.h  
  motion.h
  )

sugar_files(DRISHTI_GEOMETRY_UT
  ut/test-drishti-geometry.cpp
  )