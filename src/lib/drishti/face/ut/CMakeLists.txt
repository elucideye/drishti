set(test_name DrishtiFaceTest)
set(test_app test-drishti-face)
if(IOS)
  set(test_app_type "MACOSX_BUNDLE")
endif()

add_executable(${test_app} ${test_app_type} test-drishti-face.cpp)
target_link_libraries(${test_app} drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_property(TARGET ${test_app} PROPERTY FOLDER "app/tests")

# OS X Fix: "_CVPixelBufferLockBaseAddress", referenced from: (etc)
if(APPLE AND NOT IOS)
  target_link_libraries(${test_app}
    "-framework ImageIO"
    "-framework CoreVideo"
    )
endif()

include(drishti_set_unit_test_assets)
drishti_set_unit_test_assets(
  DRISHTI_ACF_FACE_MODEL
  DRISHTI_FACE_LANDMARKER
  DRISHTI_EYE_MODEL
  DRISHTI_MEAN_FACE_LANDMARKS
  )

enable_testing()

if(ANDROID OR IOS)

  # Extract filenames:
  get_filename_component(DRISHTI_ACF_FACE_MODEL_ "${DRISHTI_ACF_FACE_MODEL}" NAME)
  get_filename_component(DRISHTI_FACE_LANDMARKER_ "${DRISHTI_FACE_LANDMARKER}" NAME)
  get_filename_component(DRISHTI_EYE_MODEL_ "${DRISHTI_EYE_MODEL}" NAME)
  get_filename_component(DRISHTI_MEAN_FACE_LANDMARKS_ "${DRISHTI_MEAN_FACE_LANDMARKS}" NAME)

  if(ANDROID)

    # Will set ${test_name}_INSTALL_PATH
    drishti_android_add_test(
      NAME ${test_name}
      COMMAND ${test_app}
      "${DRISHTI_ACF_FACE_MODEL_}"
      "${DRISHTI_MEAN_FACE_LANDMARKS_}"
      "${DRISHTI_FACE_LANDMARKER_}"
      "${DRISHTI_EYE_MODEL_}"
      DEPENDENCIES
      "${DRISHTI_ACF_FACE_MODEL}"
      "${DRISHTI_FACE_LANDMARKER}"
      "${DRISHTI_EYE_MODEL}"
      "${DRISHTI_MEAN_FACE_LANDMARKS}"
      )
    
  elseif(IOS)

    set_target_properties(
      "${test_app}"
      PROPERTIES
      MACOSX_BUNDLE_INFO_PLIST
      "${CMAKE_SOURCE_DIR}/cmake/templates/Info.plist.in"
      XCODE_ATTRIBUTE_BUNDLE_IDENTIFIER
      "com.example.tests.drishti.face"      
      XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY "1,2" # iPhone/iPad
      )    
    
    drishti_ios_add_test(
      NAME ${test_name}
      COMMAND ${test_app}
      "~/Documents/${DRISHTI_ACF_FACE_MODEL_}"
      "~/Documents/${DRISHTI_MEAN_FACE_LANDMARKS_}"
      "~/Documents/${DRISHTI_FACE_LANDMARKER_}"
      "~/Documents/${DRISHTI_EYE_MODEL_}"
      DEPENDENCIES
      "${DRISHTI_ACF_FACE_MODEL}"
      "${DRISHTI_FACE_LANDMARKER}"
      "${DRISHTI_EYE_MODEL}"
      "${DRISHTI_MEAN_FACE_LANDMARKS}"
      CLEAN
      )
  endif()
  
else()

  add_test(NAME ${test_name} COMMAND ${test_app}
    "${DRISHTI_ACF_FACE_MODEL}"
    "${DRISHTI_MEAN_FACE_LANDMARKS}"
    "${DRISHTI_FACE_LANDMARKER}"
    "${DRISHTI_EYE_MODEL}"
    )

endif()

