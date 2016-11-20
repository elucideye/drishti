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
  XGBoosterIOArchiveBoost.cpp
  #XGBoosterIOArchiveCereal.cpp
  )

sugar_files(DRISHTI_ML_HDRS_PUBLIC
  ObjectDetector.h
  PCA.h
  RegressionTreeEnsembleShapeEstimator.h
  ShapeEstimator.h
  drishti_ml.h
  shape_predictor.h
  XGBooster.h
  XGBoosterImpl.h  
  Booster.h
  )
