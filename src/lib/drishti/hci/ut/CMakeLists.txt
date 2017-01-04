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
target_link_libraries(${test_app}
  ${OpenCV_LIBS}  
  GTest::gtest
  drishtisdk  
  ${DRISHTI_OPENGL_LIBS}
  )

### POST_BUILD GPU ADDITIONS ###
if(DRISHTI_HCI_DO_GPU)
  find_package(Qt5PrintSupport REQUIRED)
  target_link_libraries(${test_app} ${OGLES_GPGPU_LIB} Qt5::Widgets Qt5::PrintSupport)
  target_compile_definitions(${test_app} PUBLIC DRISHTI_HCI_DO_GPU=${DRISHTI_HCI_DO_GPU})
endif()

# OS X Fix: "_CVPixelBufferLockBaseAddress", referenced from: (etc)
if(APPLE)
  if(IOS)
    target_link_libraries(${test_app}
      "-framework CoreFoundation"
      "-framework Foundation"  # _NSLog
      )
  else()
    target_link_libraries(${test_app}
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

enable_testing()

if(ANDROID OR IOS)

  # Extract filenames:
  get_filename_component(DRISHTI_ACF_FACE_MODEL_ "${DRISHTI_ACF_FACE_MODEL}" NAME)
  get_filename_component(DRISHTI_FACE_LANDMARKER_ "${DRISHTI_FACE_LANDMARKER}" NAME)
  get_filename_component(DRISHTI_EYE_MODEL_ "${DRISHTI_EYE_MODEL}" NAME)
  get_filename_component(DRISHTI_MEAN_FACE_LANDMARKS_ "${DRISHTI_MEAN_FACE_LANDMARKS}" NAME)
  get_filename_component(DRISHTI_TEST_FACE_IMAGE_COLOR_ "${DRISHTI_TEST_FACE_IMAGE_COLOR}" NAME)

  if(ANDROID)

    # Will set ${test_name}_INSTALL_PATH
    drishti_android_add_test(
      NAME ${test_name}
      COMMAND ${test_app}
      "${DRISHTI_ACF_FACE_MODEL_}"
      "${DRISHTI_MEAN_FACE_LANDMARKS_}"
      "${DRISHTI_FACE_LANDMARKER_}"
      "${DRISHTI_EYE_MODEL_}"
      "${DRISHTI_TEST_FACE_IMAGE_COLOR_}"
      DEPENDENCIES
      "${DRISHTI_ACF_FACE_MODEL}"
      "${DRISHTI_FACE_LANDMARKER}"
      "${DRISHTI_EYE_MODEL}"
      "${DRISHTI_MEAN_FACE_LANDMARKS}"
      "${DRISHTI_TEST_FACE_IMAGE_COLOR}"
      )

  elseif(IOS)

    set(test_bundle_id "com.example.tests.drishti.hci")
    set_target_properties(
      "${test_app}"
      PROPERTIES
      MACOSX_BUNDLE_INFO_PLIST
      "${CMAKE_SOURCE_DIR}/cmake/templates/Info.plist.in"
      XCODE_ATTRIBUTE_BUNDLE_IDENTIFIER    
      "${test_bundle_id}"
      XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY "1,2" # iPhone/iPad
      )        
    
    drishti_ios_add_test(
      NAME ${test_name}
      BUNDLE_ID "${test_bundle_id}"      
      COMMAND ${test_app}
      "~/Documents/${DRISHTI_ACF_FACE_MODEL_}"
      "~/Documents/${DRISHTI_MEAN_FACE_LANDMARKS_}"
      "~/Documents/${DRISHTI_FACE_LANDMARKER_}"
      "~/Documents/${DRISHTI_EYE_MODEL_}"
      "~/Documents/${DRISHTI_TEST_FACE_IMAGE_COLOR_}"
      DEPENDENCIES
      "${DRISHTI_ACF_FACE_MODEL}"
      "${DRISHTI_FACE_LANDMARKER}"
      "${DRISHTI_EYE_MODEL}"
      "${DRISHTI_MEAN_FACE_LANDMARKS}"
      "${DRISHTI_TEST_FACE_IMAGE_COLOR}"
      )

  endif()
  
else()    

  add_test(NAME ${test_name} COMMAND ${test_app}
    "${DRISHTI_ACF_FACE_MODEL}"
    "${DRISHTI_MEAN_FACE_LANDMARKS}"
    "${DRISHTI_FACE_LANDMARKER}"
    "${DRISHTI_EYE_MODEL}"
    "${DRISHTI_TEST_FACE_IMAGE_COLOR}"    
    )

endif()


