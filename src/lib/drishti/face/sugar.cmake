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
  FaceArchiveCereal.cpp  
  FaceDetector.cpp
  FaceDetectorAndTracker.cpp
  FaceDetectorAndTrackerImpl.cpp
  FaceDetectorAndTrackerNN.cpp
  FaceDetectorFactory.cpp
  FaceDetectorFactoryCereal.cpp
  FaceDetectorFactoryJson.cpp  
  FaceIO.cpp
  FaceMesh.cpp
  FaceModelEstimator.cpp
  FaceTracker.cpp  
  face_util.cpp
  )

sugar_files(DRISHTI_FACE_HDRS_PUBLIC
  Face.h
  FaceDetector.h
  FaceDetectorAndTracker.h
  FaceDetectorAndTrackerImpl.h
  FaceDetectorAndTrackerNN.h
  FaceDetectorFactory.h
  FaceDetectorFactoryJson.h  
  FaceIO.h
  FaceImpl.h  
  FaceMesh.h
  FaceModelEstimator.h
  FaceTracker.h
  drishti_face.h
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
    FaceMeshMapper.cpp
    FaceMeshMapperLandmark.cpp
    FaceMeshMapperLandmarkContour.cpp
    )
  sugar_files(DRISHTI_FACE_HDRS_PUBLIC
    FaceMeshMapper.h
    FaceMeshMapperLandmark.h
    FaceMeshMapperLandmarkContour.h
    )
endif()


sugar_files(DRISHTI_FACE_UT
  ut/test-drishti-face.cpp
  )