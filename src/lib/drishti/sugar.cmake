# This file generated automatically by:
#   generate_sugar_files.py
# see wiki for more info:
#   https://github.com/ruslo/sugar/wiki/Collecting-sources

if(DEFINED SRC_LIB_DRISHTI_SUGAR_CMAKE_)
  return()
else()
  set(SRC_LIB_DRISHTI_SUGAR_CMAKE_ 1)
endif()

include(sugar_include)

sugar_include(testlib) # for testing

sugar_include(drishti) # public API
sugar_include(graphics)
sugar_include(core)
sugar_include(geometry)
sugar_include(eye)
sugar_include(ml)
sugar_include(rcpr)
sugar_include(sensor)
if(DRISHTI_BUILD_FACE)
  sugar_include(face)
endif()
# if(DRISHTI_BUILD_ACF)
#   sugar_include(acf)
# endif()
if(DRISHTI_BUILD_HCI)
  sugar_include(hci)
endif()

