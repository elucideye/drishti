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

if(BUILD_OGLES_GPGPU)
  set(DRISHTI_EYE_GPU_HDRS
    gpu/EllipsoPolarWarp.h
    gpu/TriangleStripWarp.h
    )
  set(DRISHTI_EYE_GPU_SRCS
    gpu/EllipsoPolarWarp.cpp    
    gpu/TriangleStripWarp.cpp
    )
endif()

sugar_files(DRISHTI_EYE_SRCS
  Eye.cpp
  EyeIO.cpp
  EyeModelEstimator.cpp
  EyeModelIris.cpp
  EyeModelEyelids.cpp
  EyeModelPupil.cpp
  NormalizedIris.cpp
  IrisNormalizer.cpp
  ${DRISHTI_EYE_GPU_SRCS}
  )

sugar_files(DRISHTI_EYE_HDRS_PUBLIC
  Eye.h
  EyeIO.h
  EyeModelEstimator.h
  EyeModelEstimatorImpl.h
  NormalizedIris.h
  IrisNormalizer.h
  drishti_eye.h
  ${DRISHTI_EYE_GPU_HDRS}  
  )
