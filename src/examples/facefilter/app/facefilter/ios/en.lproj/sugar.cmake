if(DEFINED APP_FACEFILTER_IOS_EN_LPROJ_SUGAR_CMAKE_)
  return()
else()
  set(APP_FACEFILTER_IOS_EN_LPROJ_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)

sugar_files(
    FACEFILTER_XIB_SOURCES
    LaunchScreen.storyboard
    FaceFilter.storyboard
)
