# found variable?
#include_directories(${GTEST_INCLUDE_DIRS})

set(DRISHTI_ML_TEST_NAME DrishtiMlTest)
set(DRISHTI_ML_TEST_APP test-drishti-ml)

add_executable(${DRISHTI_ML_TEST_APP} test-drishti-ml.cpp test-RTEShapeEstimator.cpp)
target_link_libraries(${DRISHTI_ML_TEST_APP} drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_property(TARGET ${DRISHTI_ML_TEST_APP} PROPERTY FOLDER "app/tests")

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

if(ANDROID)

  # Extract filenames:
  get_filename_component(DRISHTI_FACE_LANDMARKER_ "${DRISHTI_FACE_LANDMARKER}" NAME)
  get_filename_component(DRISHTI_TEST_FACE_ "${DRISHTI_TEST_FACE}" NAME)
  get_filename_component(DRISHTI_TEST_FACE_MODEL_ "${DRISHTI_TEST_FACE_MODEL}" NAME)

  # Will set ${DRISHTI_ML_TEST_NAME}_INSTALL_PATH
  drishti_android_add_test(NAME ${DRISHTI_ML_TEST_NAME}
    COMMAND ${DRISHTI_ML_TEST_APP}
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

else()
  add_test(NAME ${DRISHTI_ML_TEST_NAME} COMMAND ${DRISHTI_ML_TEST_APP}
    "${DRISHTI_FACE_LANDMARKER}"
    "${DRISHTI_TEST_FACE}"    
    "${DRISHTI_TEST_FACE_MODEL}"
    "${CMAKE_CURRENT_BINARY_DIR}"
    "0"
    )
endif()
