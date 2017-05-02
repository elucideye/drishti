### lib ###

set(test_lib drishti_videoio)

set(videoio_srcs
  VideoSourceCV.h
  VideoSourceCV.cpp
  VideoSourceStills.h
  VideoSourceStills.cpp
  MICMSampleBuffer.h
  MICMSampleBuffer.m
  MIMovieVideoSampleAccessor.h
  MIMovieVideoSampleAccessor.m
  )

if(APPLE)
  list(APPEND videoio_srcs
    VideoSourceApple.h
    VideoSourceApple.mm
    )
endif()

add_library(drishti_videoio ${videoio_srcs})

target_compile_options(drishti_videoio PUBLIC "-fobjc-arc")

target_include_directories(drishti_videoio
  PUBLIC
  "$<BUILD_INTERFACE:${DRISHTI_INCLUDE_DIRECTORIES}>"
  )

target_link_libraries(drishti_videoio PUBLIC ${OpenCV_LIBS})

if(APPLE)
  target_link_libraries(drishti_videoio PUBLIC
    "-framework AVFoundation"
    "-framework CoreFoundation"
    "-framework Foundation"
    "-framework CoreVideo"
    "-framework CoreMedia"
    )
  )

set_property(TARGET drishti_videoio PROPERTY FOLDER "libs/drishti")

### app ###

add_executable(videoio_test videoio.cpp)
target_link_libraries(videoio_test drishti_videoio)
set_property(TARGET videoio_test PROPERTY FOLDER "app/console")
