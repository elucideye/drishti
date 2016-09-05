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

project(drishti_rcpr)

if(NOT DRISHTI_BUILD_MIN_SIZE)
  set(DRISHTI_RCPR_OPT_SOURCE
    cprTrain.cpp
    )
endif()

sugar_files(DRISHTI_RCPR_SRCS
  CPR.cpp
  CPRIO.cpp
  cprApply.cpp
  poseGt.cpp
  "${DRISHTI_RCPR_OPT_SOURCE}"
  )

sugar_files(DRISHTI_RCPR_HDRS_PUBLIC
  CPR.h
  CPRIO.h
  Regressor.h
  RegressorXGBoost.h
  drishti_rcpr.h
  )


