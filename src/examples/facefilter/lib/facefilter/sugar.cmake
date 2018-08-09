if(DEFINED LIB_FACEFILTER_SUGAR_CMAKE_)
  return()
else()
  set(LIB_FACEFILTER_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)
include(sugar_include)

sugar_include(bindings)
sugar_include(renderer)
