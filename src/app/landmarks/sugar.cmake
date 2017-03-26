#   generate_sugar_files.py
# see wiki for more info:
#   https://github.com/ruslo/sugar/wiki/Collecting-sources

if(DEFINED SRC_APP_LANDMARKS_SUGAR_CMAKE_)
  return()
else()
  set(SRC_APP_LANDMARKS_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)

sugar_files(DRISHTI_LANDMARKS_LIB_SRCS
  BIOID.cpp
  DRISHTI.cpp
  FACE.cpp
  HELEN.cpp
  LFW.cpp
  LFPW.cpp
  MUCT.cpp
  TWO.cpp
  )

sugar_files(DRISHTI_LANDMARKS_LIB_HDRS
  BIOID.h
  DRISHTI.h  
  FACE.h  
  HELEN.h 
  LFW.h   
  LFPW.h   
  MUCT.h
  TWO.h
  )

