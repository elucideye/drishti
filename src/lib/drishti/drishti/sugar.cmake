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
  EyeSegmenter.cpp
  EyeSegmenterImpl.cpp
  Image.cpp
  drishti_sdk.cpp
  )

sugar_files(DRISHTI_DRISHTI_HDRS_PUBLIC
  Eye.hpp
  EyeSegmenter.hpp
  Image.hpp
  drishti_cv.hpp
  drishti_sdk.hpp
  drishti_gl.hpp
  ### Exclude private header from install
  ### This could be included in the IDE headers
  # EyeSegmenterImpl.hpp
  )

if(DRISHTI_BUILD_ACF)
  sugar_files(DRISHTI_DRISHTI_SRCS
    EyeDetector.cpp)
  sugar_files(DRISHTI_DRISHTI_HDRS_PUBLIC
    EyeDetector.hpp)
endif()

if(DRISHTI_BUILD_HCI)
  sugar_files(DRISHTI_DRISHTI_SRCS
    FaceTracker.cpp
    Manager.cpp
    )
  sugar_files(DRISHTI_DRISHTI_HDRS_PUBLIC
    Face.hpp
    FaceTracker.hpp    
    VideoFrame.hpp
    Manager.hpp
    ManagerImpl.h # private
    drishti_gl.hpp    
    )
endif()



