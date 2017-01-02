set(DRISHTI_TEST_NAME DrishtiDrishtiEyeTest)
set(DRISHTI_TEST_APP test-drishti-drishti-eye)

add_executable(${DRISHTI_TEST_APP} test-EyeSegmenter.cpp)
set_property(TARGET ${DRISHTI_TEST_APP} PROPERTY FOLDER "app/tests")
target_link_libraries(${DRISHTI_TEST_APP} drishti
  ${OpenCV_LIBS}
  GTest::gtest
  drishtisdk # include drishtisdk for drishti::eye::EyeModel serialization
  )
if(DRISHTI_HAVE_REGEX_H)
  target_compile_definitions(test-drishti-drishti-eye PUBLIC DRISHTI_HAVE_REGEX_H)
endif()

add_executable(test-drishti-drishti-face test-FaceTracker.cpp)
set_property(TARGET test-drishti-drishti-face PROPERTY FOLDER "app/tests")
target_link_libraries(test-drishti-drishti-face drishti
  ${OpenCV_LIBS}
  GTest::gtest
  drishtisdk # include drishtisdk for drishti::eye::EyeModel serialization
  )
if(DRISHTI_HAVE_REGEX_H)
  target_compile_definitions(test-drishti-drishti-face PUBLIC DRISHTI_HAVE_REGEX_H)
endif()

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

set(DRISHTI_TEST_EYE_IMAGE "${assets_dir}/images/2318-eye.png")
set(DRISHTI_TEST_EYE_JSON "${assets_dir}/images/2318-eye.json")    

if(ANDROID OR IOS)
  
  # Extract filenames:
  get_filename_component(DRISHTI_TEST_EYE_IMAGE_ "${DRISHTI_TEST_EYE_IMAGE}" NAME)
  get_filename_component(DRISHTI_TEST_EYE_JSON_ "${DRISHTI_TEST_EYE_JSON}" NAME) 
  get_filename_component(DRISHTI_EYE_MODEL_ "${DRISHTI_EYE_MODEL}" NAME)

  if(ANDROID)

    # Will set ${DRISHTI_TEST_NAME}_INSTALL_PATH
    drishti_android_add_test(
      NAME ${DRISHTI_TEST_NAME}
      COMMAND ${DRISHTI_TEST_APP}
      "${DRISHTI_EYE_MODEL_}"
      "${DRISHTI_TEST_EYE_IMAGE_}"
      "${DRISHTI_TEST_EYE_JSON_}"
      "." # output to install path (current working directory)
      "0"
      DEPENDENCIES # ${DRISHTI_TEST_DATA}
      "${DRISHTI_EYE_MODEL}"
      "${DRISHTI_TEST_EYE_IMAGE}"
      "${DRISHTI_TEST_EYE_JSON}"
      )

    # TODO: Make $<TARGET_FILE:drishti> work in drishti_android_add_test
    add_custom_command(
      TARGET ${DRISHTI_TEST_APP}
      POST_BUILD
      COMMAND adb shell "mkdir -p ${${DRISHTI_TEST_NAME}_INSTALL_PATH}/"    
      COMMAND adb push $<TARGET_FILE:drishti> "${${DRISHTI_TEST_NAME}_INSTALL_PATH}/"
      )

  elseif(IOS)
    message(WARNING "TODO: add ${DRISHTI_TEST_APP} for IOS")

    set(DRISHTI_TEST_BUNDLE_IDENTIFIER "com.elucideye.drishti.drishti")
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
      "${DRISHTI_EYE_MODEL_}"
      "${DRISHTI_TEST_EYE_IMAGE_}"
      "${DRISHTI_TEST_EYE_JSON_}"
      "." # output to install path (current working directory)
      "0"
      DEPENDENCIES # ${DRISHTI_TEST_DATA}
      "${DRISHTI_EYE_MODEL}"
      "${DRISHTI_TEST_EYE_IMAGE}"
      "${DRISHTI_TEST_EYE_JSON}"
      )
    
  endif()

else()    

  add_test(NAME ${DRISHTI_TEST_NAME} COMMAND ${DRISHTI_TEST_APP}
    "${DRISHTI_EYE_MODEL}"
    "${DRISHTI_TEST_EYE_IMAGE}"
    "${DRISHTI_TEST_EYE_JSON}"
    "${DRISHTI_TXT_ARCHIVE}"
    )

  if(WIN32)
    set_property(
      TEST ${DRISHTI_TEST_NAME}
      PROPERTY
      ENVIRONMENT
      "PATH=$<TARGET_FILE_DIR:drishti>;$ENV{PATH}"
      )
  endif()

endif()
