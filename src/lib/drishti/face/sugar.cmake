# This file generated automatically by:
#   generate_sugar_files.py
# see wiki for more info:
#   https://github.com/ruslo/sugar/wiki/Collecting-sources

if(DEFINED LIB_FACE_FACE_SUGAR_CMAKE_)
  return()
else()
  set(LIB_FACE_FACE_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)

sugar_files(DRISHTI_FACE_SRCS
  Face.cpp
  FaceDetector.cpp
  FaceDetectorAndTracker.cpp
  FaceDetectorAndTrackerImpl.cpp
  FaceDetectorAndTrackerNN.cpp
  FaceDetectorFactory.cpp
  FaceDetectorFactoryCereal.cpp
  FaceIO.cpp
  FaceModelEstimator.cpp
  face_util.cpp
  )

if(DRISHTI_SERIALIZE_WITH_BOOST)
  sugar_files(DRISHTI_FACE_SRCS FaceArchiveBoost.cpp)
endif()

if(DRISHTI_SERIALIZE_WITH_CEREAL OR DRISHTI_SERIALIZE_MODELS_WITH_CEREAL)
  sugar_files(DRISHTI_FACE_SRCS FaceArchiveCereal.cpp)
endif()

sugar_files(DRISHTI_FACE_HDRS_PUBLIC
  drishti_face.h
  Face.h
  FaceImpl.h  
  FaceDetector.h
  FaceDetectorAndTracker.h
  FaceDetectorAndTrackerImpl.h
  FaceDetectorAndTrackerNN.h
  FaceDetectorFactory.h
  FaceIO.h
  FaceModelEstimator.h
  face_util.h
  )

if(DRISHTI_BUILD_OGLES_GPGPU)
  sugar_Files(DRISHTI_FACE_HDRS_PUBLIC  
    gpu/EyeFilter.h
    gpu/FaceStabilizer.h
    gpu/MultiTransformProc.h
    )

  sugar_files(DRISHTI_FACE_SRCS  
    gpu/EyeFilter.cpp
    gpu/FaceStabilizer.cpp
    gpu/MultiTransformProc.cpp
    )
endif()

if(DRISHTI_BUILD_EOS)
  sugar_files(DRISHTI_FACE_SRCS
    FaceLandmarkMeshMapper.cpp)
  sugar_files(DRISHTI_FACE_HDRS_PUBLIC
    FaceLandmarkMeshMapper.h)
endif()
