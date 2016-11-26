# This file generated automatically by:
#   generate_sugar_files.py
# see wiki for more info:
#   https://github.com/ruslo/sugar/wiki/Collecting-sources

if(DEFINED LIB_ML_ML_SUGAR_CMAKE_)
  return()
else()
  set(LIB_ML_ML_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)

sugar_files(DRISHTI_ML_SRCS
  ObjectDetector.cpp
  PCA.cpp
  RegressionTreeEnsembleShapeEstimator.cpp
  ShapeEstimator.cpp
  XGBooster.cpp
  )

if(DRISHTI_SERIALIZE_WITH_BOOST)
  sugar_files(DRISHTI_ML_SRCS
    PCAArchiveBoost.cpp
    XGBoosterIOArchiveBoost.cpp    
    RTEShapeEstimatorArchiveBoost.cpp
    )
endif()

if(DRISHTI_SERIALIZE_WITH_CEREAL)
  sugar_files(DRISHTI_ML_SRCS
    PCAArchiveCereal.cpp
    XGBoosterIOArchiveCereal.cpp
    RTEShapeEstimatorArchiveCereal.cpp
  )
endif()

sugar_files(DRISHTI_ML_HDRS_PUBLIC
  ObjectDetector.h
  PCA.h
  PCAImpl.h
  RegressionTreeEnsembleShapeEstimator.h
  RTEShapeEstimatorImpl.h
  ShapeEstimator.h
  drishti_ml.h
  shape_predictor.h
  XGBooster.h
  XGBoosterImpl.h  
  Booster.h
  )
