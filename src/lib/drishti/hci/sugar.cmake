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
  gpu/LineDrawing.cpp
  gpu/GLCircle.cpp  
  gpu/GLPrinter.cpp
  gpu/FacePainter.cpp
  gpu/FlashFilter.cpp
  )

sugar_files(DRISHTI_HCI_HDRS_PUBLIC
  EyeBlob.h
  FaceFinder.h
  FaceFinderPainter.h
  FaceMonitor.h
  GazeEstimator.h
  Scene.hpp
  gpu/LineDrawing.hpp
  gpu/GLCircle.h
  gpu/GLPrinter.h
  gpu/FacePainter.h
  gpu/FlashFilter.h
  )
