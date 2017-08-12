# This file generated automatically by:
#   generate_sugar_files.py
# see wiki for more info:
#   https://github.com/ruslo/sugar/wiki/Collecting-sources

if(DEFINED LIB_RCPR_RCPR_SUGAR_CMAKE_)
  return()
else()
  set(LIB_RCPR_RCPR_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)

sugar_files(DRISHTI_RCPR_SRCS
  CPR.cpp
  CPRIOArchiveCereal.cpp  
  cprApply.cpp
  poseGt.cpp
  )

if(NOT DRISHTI_BUILD_MIN_SIZE)
  sugar_files(DRISHTI_RCPR_SRCS cprTrain.cpp)
endif()

sugar_files(DRISHTI_RCPR_HDRS_PUBLIC
  CPR.h
  CPRIO.h
  CPRIOArchive.h
  ImageMaskPair.h
  PointHalf.h
  Regressor.h
  RegressorXGBoost.h
  Vector1d.h
  drishti_rcpr.h
  )
