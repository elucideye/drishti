# Local landmark library 

set(drishti_landmark_srcs
  BIOID.cpp
  DRISHTI.cpp
  FACE.cpp
  HELEN.cpp
  LFW.cpp
  LFPW.cpp
  MUCT.cpp
  TWO.cpp  
  )

set(drishti_landmark_hdrs
  BIOID.h
  DRISHTI.h
  FACE.h
  HELEN.h
  LFW.h
  LFPW.h
  MUCT.h
  TWO.h
  )

add_library(drishti_landmarks STATIC ${drishti_landmark_srcs} ${drishti_landmark_hdrs})
target_include_directories(drishti_landmarks PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>)
set_property(TARGET drishti_landmarks PROPERTY FOLDER "libs/drishti")
