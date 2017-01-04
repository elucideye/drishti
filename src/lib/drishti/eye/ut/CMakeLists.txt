set(DRISHTI_TEST_NAME DrishtiEyeTest)
set(DRISHTI_TEST_APP test-drishti-eye)
if(IOS)
  set(TEST_APP_TYPE "MACOSX_BUNDLE")
endif()

add_executable(${DRISHTI_TEST_APP} ${TEST_APP_TYPE} test-drishti-eye.cpp test-EyeModelEstimator.cpp)
target_link_libraries(${DRISHTI_TEST_APP} drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_property(TARGET ${DRISHTI_TEST_APP} PROPERTY FOLDER "app/tests")

include(drishti_set_unit_test_assets)
drishti_set_unit_test_assets(
    DRISHTI_ACF_FACE_MODEL
    DRISHTI_FACE_LANDMARKER
    DRISHTI_EYE_MODEL
    DRISHTI_MEAN_FACE_LANDMARKS
    )

#######################################################################
message("DRISHTI_EYE_MODEL : ${DRISHTI_EYE_MODEL}")

# Legacy
if(MSVC)
  set(DRISHTI_TXT_ARCHIVE 1)
else()
  set(DRISHTI_TXT_ARCHIVE 0)
endif()

enable_testing()

set(DRISHTI_TEST_EYE "${assets_dir}/images/2318-eye.png")
set(DRISHTI_TEST_EYE_MODEL_PRIVATE "${assets_dir}/images/2318-eye_private.json")
message("DRISHTI_TEST_EYE_MODEL_PRIVATE ${DRISHTI_TEST_EYE_MODEL_PRIVATE}")

if(ANDROID OR IOS)
  
  # Extract filenames:
  get_filename_component(DRISHTI_TEST_EYE_ "${DRISHTI_TEST_EYE}" NAME)
  get_filename_component(DRISHTI_EYE_MODEL_ "${DRISHTI_EYE_MODEL}" NAME)
  get_filename_component(DRISHTI_TEST_EYE_MODEL_PRIVATE_ "${DRISHTI_TEST_EYE_MODEL_PRIVATE}" NAME) 

  if(ANDROID)
    
    drishti_android_add_test(NAME ${DRISHTI_TEST_NAME}
      COMMAND ${DRISHTI_TEST_APP}
      "${DRISHTI_EYE_MODEL_}"
      "${DRISHTI_TEST_EYE_}"
      "${DRISHTI_TEST_EYE_MODEL_PRIVATE_}"
      "." # output to install path (current working directory)
      "0"
      DEPENDENCIES
      "${DRISHTI_EYE_MODEL}"
      "${DRISHTI_TEST_EYE}"
      "${DRISHTI_TEST_EYE_MODEL_PRIVATE}"
      )
    
  elseif(IOS)

    set(DRISHTI_TEST_BUNDLE_IDENTIFIER "com.elucideye.drishti.eye")
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
      NAME "${DRISHTI_TEST_NAME}"
      BUNDLE_ID "${DRISHTI_TEST_BUNDLE_IDENTIFIER}"
      COMMAND "${DRISHTI_TEST_APP}"
      "~/Documents/${DRISHTI_EYE_MODEL_}"
      "~/Documents/${DRISHTI_TEST_EYE_}"
      "~/Documents/${DRISHTI_TEST_EYE_MODEL_PRIVATE_}"
      "~/Documents/" # output to install path (current working directory)
      "0"
      DEPENDENCIES
      "${DRISHTI_EYE_MODEL}"
      "${DRISHTI_TEST_EYE}"
      "${DRISHTI_TEST_EYE_MODEL_PRIVATE}"
      )
  endif()

else()

  add_test(NAME ${DRISHTI_TEST_NAME} COMMAND ${DRISHTI_TEST_APP}
    "${DRISHTI_EYE_MODEL}"
    "${DRISHTI_TEST_EYE}"
    "${DRISHTI_TEST_EYE_MODEL_PRIVATE}"
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${DRISHTI_TXT_ARCHIVE}"
    )

endif()

