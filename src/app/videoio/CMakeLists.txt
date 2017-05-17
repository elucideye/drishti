### lib ###

set(test_lib drishti_videoio)

set(videoio_srcs
  VideoSourceCV.h
  VideoSourceCV.cpp
  VideoSourceStills.h
  VideoSourceStills.cpp
  )

if(APPLE)
  list(APPEND videoio_srcs
    # AVFoundation framework video (i.e., quicktime *.MOV)
    VideoSourceApple.h
    VideoSourceApple.mm
    MICMSampleBuffer.h
    MICMSampleBuffer.m
    MIMovieVideoSampleAccessor.h
    MIMovieVideoSampleAccessor.m    
    )
endif()

add_library(drishti_videoio ${videoio_srcs})

target_include_directories(drishti_videoio
  PUBLIC
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

set_target_properties(drishti_videoio PROPERTIES FOLDER "libs/drishti")

### app ###

add_executable(test-videoio videoio.cpp)
target_link_libraries(test-videoio drishti_videoio cxxopts::cxxopts)
set_target_properties(test-videoio PROPERTIES FOLDER "app/console" INTERPROCEDURAL_OPTIMIZATION ${DRISHTI_EXE_IPO})
