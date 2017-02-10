# This file generated automatically by:
#   generate_sugar_files.py
# see wiki for more info:
#   https://github.com/ruslo/sugar/wiki/Collecting-sources

if(DEFINED LIB_GRAPHICS_GRAPHICS_SUGAR_CMAKE_)
  return()
else()
  set(LIB_GRAPHICS_GRAPHICS_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)

#sugar_files(
#  DRISHTI_GRAPHICS_SRCS
#  )

sugar_files(
  DRISHTI_GRAPHICS_HDRS_PUBLIC
  drishti_graphics.h
  )

if(DRISHTI_BUILD_OGLES_GPGPU)
  sugar_files(
    DRISHTI_GRAPHICS_SRCS
    binomial.cpp
    fade.cpp
    gain.cpp
    rgb2hsv.cpp
    rgb2luv.cpp
    saturation.cpp
    swizzle.cpp    
    )
  sugar_files(
    DRISHTI_GRAPHICS_HDRS_PUBLIC
    binomial.h
    fade.h
    gain.h
    rgb2hsv.h
    rgb2luv.h
    saturation.h
    swizzle.h
    )
endif()


