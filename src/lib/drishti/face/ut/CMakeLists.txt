set(DRISHTI_TEST_NAME DrishtiFaceTest)
set(DRISHTI_TEST_APP test-drishti-face)
if(IOS)
  set(TEST_APP_TYPE "MACOSX_BUNDLE")
endif()

add_executable(${DRISHTI_TEST_APP} ${TEST_APP_TYPE} test-drishti-face.cpp)
target_link_libraries(${DRISHTI_TEST_APP} drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_property(TARGET ${DRISHTI_TEST_APP} PROPERTY FOLDER "app/tests")
if(DRISHTI_HAVE_REGEX_H)
  target_compile_definitions(${DRISHTI_TEST_APP} PUBLIC DRISHTI_HAVE_REGEX_H)
endif()

# OS X Fix: "_CVPixelBufferLockBaseAddress", referenced from: (etc)
if(APPLE AND NOT IOS)
  target_link_libraries(${DRISHTI_TEST_APP}
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

    # Will set ${DRISHTI_TEST_NAME}_INSTALL_PATH
    drishti_android_add_test(
      NAME ${DRISHTI_TEST_NAME}
      COMMAND ${DRISHTI_TEST_APP}
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

    set(DRISHTI_TEST_BUNDLE_IDENTIFIER "com.elucideye.drishti.face")
    set_target_properties(
      "${DRISHTI_TEST_APP}"
      PROPERTIES
      MACOSX_BUNDLE_INFO_PLIST
      "${CMAKE_SOURCE_DIR}/cmake/templates/Info.plist.in"
      XCODE_ATTRIBUTE_BUNDLE_IDENTIFIER    
      "${DRISHTI_TEST_BUNDLE_IDENTIFIER}"
      XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY "1,2" # iPhone/iPad
      )    
    
    drishti_ios_add_test(
      NAME ${DRISHTI_TEST_NAME}
      BUNDLE_ID "${DRISHTI_TEST_BUNDLE_IDENTIFIER}"      
      COMMAND ${DRISHTI_TEST_APP}
      "${DRISHTI_ACF_FACE_MODEL_}"
      "${DRISHTI_MEAN_FACE_LANDMARKS_}"
      "${DRISHTI_FACE_LANDMARKER_}"
      "${DRISHTI_EYE_MODEL_}"
      DEPENDENCIES
      "~/${DRISHTI_ACF_FACE_MODEL}"
      "~/${DRISHTI_FACE_LANDMARKER}"
      "~/${DRISHTI_EYE_MODEL}"
      "~/${DRISHTI_MEAN_FACE_LANDMARKS}"
      CLEAN
      )
  endif()
  
else()

  add_test(NAME ${DRISHTI_TEST_NAME} COMMAND ${DRISHTI_TEST_APP}
    "${DRISHTI_ACF_FACE_MODEL}"
    "${DRISHTI_MEAN_FACE_LANDMARKS}"
    "${DRISHTI_FACE_LANDMARKER}"
    "${DRISHTI_EYE_MODEL}"
    )

endif()

