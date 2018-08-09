if(DEFINED LIB_FACEFILTER_BINDINGS_SUGAR_CMAKE_)
  return()
else()
  set(LIB_FACEFILTER_BINDINGS_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)
include(sugar_include)

if(ANDROID)
  sugar_include(android)
endif()

if(IOS)
  sugar_include(ios)
endif()
