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

if(BUILD_DRISHTI_ACF)
  set(DRISHTI_DRISHTI_OPT_SRCS EyeDetector.cpp)
  set(DRISHTI_DRISHTI_OPT_HDRS EyeDetector.hpp)
endif()

sugar_files(DRISHTI_DRISHTI_SRCS
  Eye.cpp
  EyeSegmenter.cpp
  EyeSegmenterImpl.cpp  
  Image.cpp
  drishti_sdk.cpp
  ${DRISHTI_DRISHTI_OPT_SRCS}
)

sugar_files(DRISHTI_DRISTHI_HDRS_PUBLIC
  Eye.hpp
  EyeSegmenter.hpp
  EyeSegmenterImpl.hpp  
  Image.hpp
  drishti_cv.hpp
  drishti_sdk.hpp
  ${DRISHTI_DRISHTI_OPT_HDRS}
)


