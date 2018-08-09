if(DEFINED LIB_FACEFILTER_BINDINGS_ANDROID_SUGAR_CMAKE_)
  return()
else()
  set(LIB_FACEFILTER_BINDINGS_ANDROID_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)

sugar_files(
    FACEFILTER_BINDINGS_SRCS
    jni.cpp
)

sugar_files(
    FACEFILTER_BINDINGS_HDRS
    Util.h
)
