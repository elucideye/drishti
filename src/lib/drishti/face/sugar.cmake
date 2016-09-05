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

# if(BUILD_CV_TRACKING)
#   set(DRISHTI_FACE_TRACKING_SRCS
# 	FaceDetectorAndTrackerCorrelation.h
# 	FaceDetectorAndTrackerCorrelation.cpp
# 	)
#   set(DRISHTI_FACE_TRACKING_HDRS_PUBLIC
# 	FaceDetectorAndTrackerCV.h
# 	FaceDetectorAndTrackerCV.cpp
# 	)
# endif()

if(DRISHTI_BUILD_OGLES_GPGPU)
  set(DRISHTI_FACE_GPU_HDRS
    gpu/EyeFilter.h
    gpu/FaceStabilizer.h
    gpu/MultiTransformProc.h # move to generic
    )
  set(DRISHTI_FACE_GPU_SRCS
    gpu/EyeFilter.cpp
    gpu/FaceStabilizer.cpp
    gpu/MultiTransformProc.cpp # move to generic
    )
endif()

sugar_files(DRISHTI_FACE_SRCS
  Face.cpp
  FaceDetector.cpp
  FaceDetectorAndTracker.cpp
  FaceDetectorAndTrackerImpl.cpp
  FaceDetectorAndTrackerNN.cpp
  FaceDetectorFactory.cpp
  FaceIO.cpp
  FaceLandmarkMeshMapper.cpp  
  FaceModelEstimator.cpp
  GazeEstimator.cpp
  face_util.cpp
  ${DRISHTI_FACE_TRACKING_SRCS}
  ${DRISHTI_FACE_GPU_HDRS}  
)

sugar_files(DRISHTI_FACE_HDRS_PUBLIC
  drishti_face.h
  Face.h
  FaceDetector.h
  FaceDetectorAndTracker.h
  FaceDetectorAndTrackerImpl.h
  FaceDetectorAndTrackerNN.h
  FaceDetectorFactory.h
  FaceIO.h
  FaceLandmarkMeshMapper.h
  FaceModelEstimator.h
  GazeEstimator.h
  face_util.h
  ${DRISHTI_FACE_TRACKING_HDRS_PUBLIC}
  ${DRISHTI_FACE_GPU_SRCS}
  )
