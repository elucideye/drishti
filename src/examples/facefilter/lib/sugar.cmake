if(DEFINED LIB_SUGAR_CMAKE_)
  return()
else()
  set(LIB_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)
include(sugar_include)

sugar_include(facefilter)
