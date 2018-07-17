if(DEFINED LIB_FACEFILTER_RENDERER_SUGAR_CMAKE_)
  return()
else()
  set(LIB_FACEFILTER_RENDERER_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)

sugar_include(android)

sugar_files(
    FACEFILTER_RENDERER_SRCS
    Context.cpp
    Renderer.cpp
    fill.cpp

    ### drishti ###
    FaceTrackerFactoryJson.cpp
    FaceTrackerTest.cpp
)

sugar_files(
    FACEFILTER_RENDERER_HDRS
    Context.h
    Renderer.h
    fill.h

    ### drishti ###
    FaceTrackerFactoryJson.h
    FaceTrackerTest.h
)
