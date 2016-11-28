# found variable?
#include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test-drishti-eye test-drishti-eye.cpp test-EyeModelEstimator.cpp)
target_link_libraries(test-drishti-eye drishtisdk ${OpenCV_LIBS} GTest::gtest)

enable_testing()

set(DRISHTI_TEST_EYE "${assets_dir}/images/2318-eye.png")
set(DRISHTI_TEST_EYE_MODEL "${assets_dir}/images/2318-eye.json")
string(REGEX REPLACE "\\.json" "_private.json" DRISHTI_TEST_EYE_MODEL_PRIVATE "${DRISHTI_TEST_EYE_MODEL}")
message("CONVERT ${DRISHTI_TEST_EYE_MODEL} => ${DRISHTI_TEST_EYE_MODEL_PRIVATE}")

if(MSVC)
  add_test(NAME DrishtiEyeTest COMMAND test-drishti-eye
    "${assets_dir}/drishti_eye_full_npd_eix.txt"
    "${DRISHTI_TEST_EYE}"
    "${DRISHTI_TEST_EYE_MODEL_PRIVATE}"
    "${CMAKE_CURRENT_BINARY_DIR}"
    "1"
    )
else()
  add_test(NAME DrishtiEyeTest COMMAND test-drishti-eye
    "${assets_dir}/drishti_eye_full_npd_eix.pba.z"
    "${DRISHTI_TEST_EYE}"    
    "${DRISHTI_TEST_EYE_MODEL_PRIVATE}"
    "${CMAKE_CURRENT_BINARY_DIR}"
    "0"
    )
endif()

set_property(TARGET test-drishti-eye PROPERTY FOLDER "app/tests")
