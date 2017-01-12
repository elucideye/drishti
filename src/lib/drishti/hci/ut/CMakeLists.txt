set(test_name DrishtiHciTest)
set(test_app test-drishti-hci)

if(DRISHTI_BUILD_QT AND DRISHTI_BUILD_OGLES_GPGPU)
  set(DRISHTI_HCI_DO_GPU 1)
else()
  set(DRISHTI_HCI_DO_GPU 0)
endif()

set(SOURCES
  test-drishti-hci.cpp
  test-hessian-cpu.cpp
  test-hessian-cpu.h
  test-FaceFinder.cpp
  FaceMonitorHCITest.h  
  )

### PRE_BUILD GPU ADDITIONS ###
if(DRISHTI_HCI_DO_GPU)
  list(APPEND SOURCES ${DRISHTI_QTPLUS_HDRS} ${DRISHTI_QTPLUS_SRCS})
endif()

add_executable(${test_app} ${SOURCES})
set_property(TARGET ${test_app} PROPERTY FOLDER "app/tests")
target_link_libraries(${test_app} PUBLIC
  ${OpenCV_LIBS}  
  GTest::gtest
  drishtisdk  
  ${DRISHTI_OPENGL_LIBS}
  )

### POST_BUILD GPU ADDITIONS ###
if(DRISHTI_HCI_DO_GPU)
  find_package(Qt5PrintSupport REQUIRED)
  target_link_libraries(${test_app} PUBLIC ${OGLES_GPGPU_LIB} Qt5::Widgets Qt5::PrintSupport)
  target_compile_definitions(${test_app} PUBLIC DRISHTI_HCI_DO_GPU=${DRISHTI_HCI_DO_GPU})
endif()

# OS X Fix: "_CVPixelBufferLockBaseAddress", referenced from: (etc)
if(APPLE)
  if(IOS)
    target_link_libraries(${test_app} PUBLIC
      "-framework CoreFoundation"
      "-framework Foundation"  # _NSLog
      )
  else()
    target_link_libraries(${test_app} PUBLIC
      "-framework ImageIO"
      "-framework CoreVideo"
      )
  endif()
endif()

include(drishti_set_unit_test_assets)
drishti_set_unit_test_assets(
  DRISHTI_ACF_FACE_MODEL
  DRISHTI_FACE_LANDMARKER
  DRISHTI_EYE_MODEL
  DRISHTI_MEAN_FACE_LANDMARKS
  )

set(DRISHTI_TEST_FACE_IMAGE_COLOR "${assets_dir}/images/lena512color.png")

drishti_add_test(NAME ${test_name} COMMAND ${test_app}
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_ACF_FACE_MODEL}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_MEAN_FACE_LANDMARKS}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_FACE_LANDMARKER}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_EYE_MODEL}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_TEST_FACE_IMAGE_COLOR}>"    
  )
