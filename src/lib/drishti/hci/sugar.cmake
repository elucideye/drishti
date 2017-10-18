# This file generated automatically by:
#   generate_sugar_files.py
# see wiki for more info:
#   https://github.com/ruslo/sugar/wiki/Collecting-sources

if(DEFINED LIB_DRISHTI_HCI_SUGAR_CMAKE_)
  return()
else()
  set(LIB_DRISHTI_HCI_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)

sugar_files(DRISHTI_HCI_SRCS
  EyeBlob.cpp
  FaceFinder.cpp
  FaceFinderPainter.cpp
  GazeEstimator.cpp
  Scene.cpp
  gpu/BlobFilter.cpp
  gpu/FacePainter.cpp
  gpu/GLCircle.cpp  
  gpu/GLPrinter.cpp
  gpu/LineDrawing.cpp
  )

sugar_files(DRISHTI_HCI_HDRS_PUBLIC
  EyeBlob.h
  FaceFinder.h
  FaceFinderImpl.h
  FaceFinderPainter.h
  FaceMonitor.h
  GazeEstimator.h
  Scene.hpp
  gpu/BlobFilter.h
  gpu/FacePainter.h
  gpu/GLCircle.h
  gpu/GLPrinter.h
  gpu/LineDrawing.hpp
  )

sugar_files(DRISHTI_HCI_UT
  ut/FaceMonitorHCITest.h
  ut/test-drishti-hci.cpp
  ut/test-hessian-cpu.cpp
  ut/test-hessian-cpu.h
  )
