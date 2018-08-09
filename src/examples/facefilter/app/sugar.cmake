if(DEFINED APP_SUGAR_CMAKE_)
  return()
else()
  set(APP_SUGAR_CMAKE_ 1)
endif()

include(sugar_include)

sugar_include(facefilter)

