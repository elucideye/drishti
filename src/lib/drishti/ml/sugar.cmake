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
  PCAArchiveCereal.cpp
  RTEShapeEstimatorArchiveCereal.cpp  
  RegressionTreeEnsembleShapeEstimator.cpp
  ShapeEstimator.cpp
  XGBooster.cpp
  XGBoosterIOArchiveCereal.cpp
  )

sugar_files(DRISHTI_ML_HDRS_PUBLIC
  Booster.h
  ObjectDetector.h
  PCA.h
  PCAImpl.h
  RTEShapeEstimatorImpl.h
  RegressionTreeEnsembleShapeEstimator.h
  ShapeEstimator.h
  XGBooster.h
  XGBoosterImpl.h  
  drishti_ml.h
  shape_predictor.h
  shape_predictor_archive.h
  )

sugar_files(DRISHTI_ML_UT
  ut/test-drishti-ml.cpp
  )

if(NOT DRISHTI_BUILD_MIN_SIZE)
  sugar_files(DRISHTI_ML_HDRS_PUBLIC shape_predictor_trainer.h)
endif()

if(DRISHTI_BUILD_DEST)
  sugar_files(DRISHTI_ML_HDRS_PUBLIC RegressionTreeEnsembleShapeEstimatorDEST.h)
  sugar_files(DRISHTI_ML_SRCS RegressionTreeEnsembleShapeEstimatorDEST.cpp)
endif()

if(DRISHTI_BUILD_CV_ML)
  sugar_files(DRISHTI_ML_HDRS_PUBLIC ObjectDetectorCV.h)
  sugar_files(DRISHTI_ML_SRCS ObjectDetectorCV.cpp)
endif()
