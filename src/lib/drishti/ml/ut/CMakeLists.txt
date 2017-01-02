set(DRISHTI_TEST_NAME DrishtiMlTest)
set(DRISHTI_TEST_APP test-drishti-ml)
if(IOS)
  set(TEST_APP_TYPE "MACOSX_BUNDLE")
endif()

add_executable(${DRISHTI_TEST_APP} ${TEST_APP_TYPE} test-drishti-ml.cpp test-RTEShapeEstimator.cpp)
target_link_libraries(${DRISHTI_TEST_APP} drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_property(TARGET ${DRISHTI_TEST_APP} PROPERTY FOLDER "app/tests")
if(DRISHTI_HAVE_REGEX_H)
  target_compile_definitions(${DRISHTI_TEST_APP} PUBLIC DRISHTI_HAVE_REGEX_H=1)
endif()

include(drishti_set_unit_test_assets)
drishti_set_unit_test_assets(
    DRISHTI_ACF_FACE_MODEL
    DRISHTI_FACE_LANDMARKER
    DRISHTI_EYE_MODEL
    DRISHTI_MEAN_FACE_LANDMARKS
    )

enable_testing()

#####

set(DRISHTI_TEST_FACE "${assets_dir}/images/lena512color.png")
set(DRISHTI_TEST_FACE_MODEL "${assets_dir}/images/lena512color.json") # missing

if(ANDROID OR IOS)

  # Extract filenames:
  get_filename_component(DRISHTI_FACE_LANDMARKER_ "${DRISHTI_FACE_LANDMARKER}" NAME)
  get_filename_component(DRISHTI_TEST_FACE_ "${DRISHTI_TEST_FACE}" NAME)
  get_filename_component(DRISHTI_TEST_FACE_MODEL_ "${DRISHTI_TEST_FACE_MODEL}" NAME)

  if(ANDROID)

    # Will set ${DRISHTI_TEST_NAME}_INSTALL_PATH
    drishti_android_add_test(NAME ${DRISHTI_TEST_NAME}
      COMMAND ${DRISHTI_TEST_APP}
      "${DRISHTI_FACE_LANDMARKER_}"
      "${DRISHTI_TEST_FACE_}"    
      "${DRISHTI_TEST_FACE_MODEL_}"
      "." # output to install path (current working directory)
      "0"
      DEPENDENCIES
      "${DRISHTI_FACE_LANDMARKER}"
      "${DRISHTI_TEST_FACE}"    
      "${DRISHTI_TEST_FACE_MODEL}"
      )

  elseif(IOS)

    set(DRISHTI_TEST_BUNDLE_IDENTIFIER "com.github.elucideye.drishti.ml") 
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
      "~/Documents/${DRISHTI_FACE_LANDMARKER_}"
      "~/Documents/${DRISHTI_TEST_FACE_}"    
      "~/Documents/${DRISHTI_TEST_FACE_MODEL_}"
      "~/Documents/"
      "0"
      DEPENDENCIES
      "${DRISHTI_FACE_LANDMARKER}"
      "${DRISHTI_TEST_FACE}"    
      "${DRISHTI_TEST_FACE_MODEL}"
      CLEAN
      )

  endif()
  
else()    
    add_test(NAME ${DRISHTI_TEST_NAME} COMMAND ${DRISHTI_TEST_APP}
      "${DRISHTI_FACE_LANDMARKER}"
      "${DRISHTI_TEST_FACE}"    
      "${DRISHTI_TEST_FACE_MODEL}"
      "${CMAKE_CURRENT_BINARY_DIR}"
      "0"
      )
  endif()
