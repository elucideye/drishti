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
  EyeIO.cpp
  EyeModelEstimator.cpp
  EyeModelIris.cpp
  EyeModelEyelids.cpp
  EyeModelPupil.cpp
  NormalizedIris.cpp
  IrisNormalizer.cpp
  )

if(DRISHTI_SERIALIZE_WITH_BOOST)
  sugar_files(DRISHTI_EYE_SRCS
    EyeModelEstimatorArchiveBoost.cpp
    EyeArchiveBoost.cpp
    )
endif()

if(DRISHTI_SERIALIZE_WITH_CEREAL)
  sugar_files(DRISHTI_EYE_SRCS
    EyeModelEstimatorArchiveCereal.cpp
    )
endif()

if(DRISHTI_SERIALIZE_WITH_CEREAL OR DRISHTI_SERIALIZE_MODELS_WITH_CEREAL)
  sugar_files(DRISHTI_EYE_SRCS
    EyeArchiveCereal.cpp    
    )
endif()

sugar_files(DRISHTI_EYE_HDRS_PUBLIC
  Eye.h
  EyeImpl.h
  EyeIO.h
  EyeModelEstimator.h
  EyeModelEstimatorImpl.h
  NormalizedIris.h
  IrisNormalizer.h
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
