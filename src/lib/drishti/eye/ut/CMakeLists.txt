#include_directories(${GTEST_INCLUDE_DIRS})

set(DRISHTI_EYE_TEST_NAME DrishtiEyeTest)
set(DRISHTI_EYE_TEST_APP test-drishti-eye)

add_executable(${DRISHTI_EYE_TEST_APP} test-drishti-eye.cpp test-EyeModelEstimator.cpp)
target_link_libraries(${DRISHTI_EYE_TEST_APP} drishtisdk ${OpenCV_LIBS} GTest::gtest)

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

if(ANDROID)
  
  # Extract filenames:
  get_filename_component(DRISHTI_TEST_EYE_ "${DRISHTI_TEST_EYE}" NAME)
  get_filename_component(DRISHTI_EYE_MODEL_ "${DRISHTI_EYE_MODEL}" NAME)
  get_filename_component(DRISHTI_TEST_EYE_MODEL_PRIVATE_ "${DRISHTI_TEST_EYE_MODEL_PRIVATE}" NAME) 

  # Will set ${DRISHTI_ML_TEST_NAME}_INSTALL_PATH
  drishti_android_add_test(NAME ${DRISHTI_EYE_TEST_NAME}
    COMMAND ${DRISHTI_EYE_TEST_APP}
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
  
else()

  add_test(NAME ${DRISHTI_EYE_TEST_NAME} COMMAND ${DRISHTI_EYE_TEST_APP}
    "${DRISHTI_EYE_MODEL}"
    "${DRISHTI_TEST_EYE}"
    "${DRISHTI_TEST_EYE_MODEL_PRIVATE}"
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${DRISHTI_TXT_ARCHIVE}"
    )

endif()

set_property(TARGET ${DRISHTI_EYE_TEST_APP} PROPERTY FOLDER "app/tests")
