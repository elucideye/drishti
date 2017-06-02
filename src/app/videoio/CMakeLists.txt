### lib ###

set(test_lib drishti_videoio)

set(videoio_srcs

  drishti_videoio.h
  
  # VideoSink:
  VideoSinkCV.h
  VideoSinkCV.cpp
  
  # VideoSource:
  VideoSourceCV.h
  VideoSourceCV.cpp
  VideoSourceStills.h
  VideoSourceStills.cpp
  VideoSourceTest.h
  VideoSourceTest.cpp
  )

if(APPLE)
  # AVFoundation framework video (i.e., quicktime *.MOV)  
  list(APPEND videoio_srcs

    # VideoSink:
    VideoSinkApple.h
    VideoSinkApple.mm

    # VideoSource:
    VideoSourceApple.h
    VideoSourceApple.mm
    MICMSampleBuffer.h
    MICMSampleBuffer.m
    MIMovieVideoSampleAccessor.h
    MIMovieVideoSampleAccessor.m    
    )
endif()

add_library(drishti_videoio ${videoio_srcs})

target_include_directories(drishti_videoio PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../>" 
  "$<BUILD_INTERFACE:${DRISHTI_INCLUDE_DIRECTORIES}>"  
  )

target_link_libraries(drishti_videoio PUBLIC ${OpenCV_LIBS} drishtisdk)

if(APPLE)
  target_compile_options(drishti_videoio PUBLIC "-fobjc-arc")  
  target_compile_definitions(drishti_videoio PUBLIC DRISHTI_USE_AVFOUNDATION=1)
  target_link_libraries(drishti_videoio PUBLIC
    "-framework AVFoundation"
    "-framework CoreFoundation"
    "-framework Foundation"
    "-framework CoreVideo"
    "-framework CoreMedia"
    )
endif()

set_property(TARGET drishti_videoio PROPERTY FOLDER "libs/drishti")

### app ###

add_executable(test-videoio videoio.cpp)
target_link_libraries(test-videoio drishti_videoio cxxopts::cxxopts)
set_property(TARGET test-videoio PROPERTY FOLDER "app/console")
