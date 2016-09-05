# This file generated automatically by:
#   generate_sugar_files.py
# see wiki for more info:
#   https://github.com/ruslo/sugar/wiki/Collecting-sources

if(DEFINED LIB_SENSOR_SENSOR_SUGAR_CMAKE_)
  return()
else()
  set(LIB_SENSOR_SENSOR_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)

sugar_files(DRISHTI_SENSOR_SRCS
  Sensor.cpp
  )

sugar_files(DRISHTI_SENSOR_HDRS_PUBLIC
  Sensor.h
  drishti_sensor.h
  )