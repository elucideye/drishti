if(DEFINED APP_FACEFILTER_IOS_SUGAR_CMAKE_)
  return()
else()
  set(APP_FACEFILTER_IOS_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)
include(sugar_include)

sugar_include(en.lproj)
sugar_include(Shaders)

sugar_files(
    FACEFILTER_SWIFT_SRCS
    AppDelegate.h
    AppDelegate.swift
    FaceFilterViewController.swift
    OpenGLRendering.swift
)
