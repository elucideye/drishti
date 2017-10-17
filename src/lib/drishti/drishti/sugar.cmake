# This file generated automatically by:
#   generate_sugar_files.py
# see wiki for more info:
#   https://github.com/ruslo/sugar/wiki/Collecting-sources

if(DEFINED LIB_DRISHTI_DRISHTI_SUGAR_CMAKE_)
  return()
else()
  set(LIB_DRISHTI_DRISHTI_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)

sugar_files(DRISHTI_DRISHTI_SRCS
  Eye.cpp
  EyeIO.cpp 
  EyeSegmenter.cpp
  EyeSegmenterImpl.cpp
  Image.cpp
  drishti_sdk.cpp
  )

sugar_files(DRISHTI_DRISHTI_HDRS_PUBLIC
  Array.hpp    
  Eye.hpp
  EyeIO.hpp  
  EyeSegmenter.hpp
  Image.hpp
  drishti_cv.hpp
  #drishti_sdk.hpp # generated file
  ### Exclude private header from install
  ### This could be included in the IDE headers
  # EyeSegmenterImpl.hpp
  )

sugar_files(DRISHTI_DRISHTI_UT
  ut/test-EyeSegmenter.cpp
  )

if(DRISHTI_BUILD_ACF)
  sugar_files(DRISHTI_DRISHTI_SRCS
    EyeDetector.cpp)
  sugar_files(DRISHTI_DRISHTI_HDRS_PUBLIC
    drishti_gl.hpp # only needed for GPU functionality
    EyeDetector.hpp
    )
endif()

if(DRISHTI_BUILD_HCI)
  sugar_files(DRISHTI_DRISHTI_SRCS
    Context.cpp
    FaceTracker.cpp
    Sensor.cpp
    )
  sugar_files(DRISHTI_DRISHTI_HDRS_PUBLIC
    Context.hpp
    Face.hpp
    FaceTracker.hpp
    Sensor.hpp
    VideoFrame.hpp
    drishti_gl.hpp    
    )

  sugar_files(DRISHTI_DRISHTI_HDRS_PRIVATE
    ContextImpl.h 
    FaceMonitorAdapter.h
    SensorImpl.h
    )

  sugar_files(DRISHTI_DRISHTI_UT  
    ut/test-FaceTracker.cpp
    )
  
endif()



