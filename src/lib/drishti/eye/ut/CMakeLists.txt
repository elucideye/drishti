#include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test-drishti-eye test-drishti-eye.cpp test-EyeModelEstimator.cpp)
target_link_libraries(test-drishti-eye drishtisdk ${OpenCV_LIBS} GTest::gtest)

include(drishti_set_unit_test_assets)
drishti_set_unit_test_assets(
    DRISHTI_ACF_FACE_MODEL
    DRISHTI_FACE_LANDMARKER
    DRISHTI_EYE_MODEL
    DRISHTI_MEAN_FACE_LANDMARKS
    )

#######################################################################
#######################################################################
#######################################################################
message("DRISHTI_ACF_FACE_MODEL : ${DRISHTI_ACF_FACE_MODEL}")
message("DRISHTI_FACE_LANDMARKER : ${DRISHTI_ACF_FACE_MODEL}")
message("DRISHTI_EYE_MODEL : ${DRISHTI_EYE_MODEL}")
message("DRISHTI_MEAN_FACE_LANDMARKS : ${DRISHTI_MEAN_FACE_LANDMARKS}")

# Legacy
if(MSVC)
  set(DRISHTI_TXT_ARCHIVE 1)
else()
  set(DRISHTI_TXT_ARCHIVE 0)
endif()

enable_testing()

set(DRISHTI_TEST_EYE "${assets_dir}/images/2318-eye.png")
set(DRISHTI_TEST_EYE_MODEL "${assets_dir}/images/2318-eye.json")
string(REGEX REPLACE "\\.json" "_private.json" DRISHTI_TEST_EYE_MODEL_PRIVATE "${DRISHTI_TEST_EYE_MODEL}")
message("CONVERT ${DRISHTI_TEST_EYE_MODEL} => ${DRISHTI_TEST_EYE_MODEL_PRIVATE}")

add_test(NAME DrishtiEyeTest COMMAND test-drishti-eye
  "${DRISHTI_EYE_MODEL}"
  "${DRISHTI_TEST_EYE}"
  "${DRISHTI_TEST_EYE_MODEL_PRIVATE}"
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${DRISHTI_TXT_ARCHIVE}"
  )

set_property(TARGET test-drishti-eye PROPERTY FOLDER "app/tests")
