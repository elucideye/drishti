# This file generated automatically by:
#   generate_sugar_files.py
# see wiki for more info:
#   https://github.com/ruslo/sugar/wiki/Collecting-sources

if(DEFINED LIB_EYE_EYE_SUGAR_CMAKE_)
  return()
else()
  set(LIB_EYE_EYE_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)

sugar_files(DRISHTI_EYE_SRCS
  Eye.cpp
  EyeArchiveCereal.cpp      
  EyeIO.cpp
  EyeModelEstimator.cpp
  EyeModelEstimatorArchiveCereal.cpp
  EyeModelEyelids.cpp
  EyeModelIris.cpp
  EyeModelPupil.cpp
  IrisNormalizer.cpp
  NormalizedIris.cpp
  )

sugar_files(DRISHTI_EYE_HDRS_PUBLIC
  Eye.h
  EyeIO.h
  EyeImpl.h
  EyeModelEstimator.h
  EyeModelEstimatorImpl.h
  IrisNormalizer.h
  NormalizedIris.h
  drishti_eye.h
  )

if(DRISHTI_BUILD_OGLES_GPGPU)
  sugar_files(DRISHTI_EYE_HDRS_PUBLIC
    gpu/EllipsoPolarWarp.h
    gpu/TriangleStripWarp.h
    )
  sugar_files(DRISHTI_EYE_SRCS
    gpu/EllipsoPolarWarp.cpp    
    gpu/TriangleStripWarp.cpp
    )
endif()

sugar_files(DRISHTI_EYE_UT
  ut/test-drishti-eye.cpp
  )
