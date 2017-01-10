set(test_name DrishtiMlTest)
set(test_app test-drishti-ml)
if(IOS)
  set(test_app_type "MACOSX_BUNDLE")
endif()

add_executable(${test_app} ${test_app_type} test-drishti-ml.cpp test-RTEShapeEstimator.cpp)
target_link_libraries(${test_app} drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_property(TARGET ${test_app} PROPERTY FOLDER "app/tests")

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

    # Will set ${test_name}_INSTALL_PATH
    drishti_android_add_test(NAME ${test_name}
      COMMAND ${test_app}
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

    set_target_properties(
      "${test_app}"
      PROPERTIES
      MACOSX_BUNDLE_INFO_PLIST
      "${CMAKE_SOURCE_DIR}/cmake/templates/Info.plist.in"
      XCODE_ATTRIBUTE_BUNDLE_IDENTIFIER
      "com.examle.tests.drishti.ml"      
      XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY "1,2" # iPhone/iPad
      )
    
    drishti_ios_add_test(
      NAME "${test_name}"
      COMMAND "${test_app}"
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
  add_test(NAME ${test_name} COMMAND ${test_app}
    "${DRISHTI_FACE_LANDMARKER}"
    "${DRISHTI_TEST_FACE}"    
    "${DRISHTI_TEST_FACE_MODEL}"
    "${CMAKE_CURRENT_BINARY_DIR}"
    "0"
    )
endif()
