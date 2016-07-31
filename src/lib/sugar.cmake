# This file generated automatically by:
#   generate_sugar_files.py
# see wiki for more info:
#   https://github.com/ruslo/sugar/wiki/Collecting-sources

if(DEFINED LIB_SUGAR_CMAKE_)
  return()
else()
  set(LIB_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)
include(sugar_include)

sugar_include(drishti) # public API
sugar_include(graphics)
sugar_include(core)
sugar_include(face)
sugar_include(geometry)
sugar_include(eye)
sugar_include(ml)
sugar_include(acf)
sugar_include(rcpr)
sugar_include(sensor)


