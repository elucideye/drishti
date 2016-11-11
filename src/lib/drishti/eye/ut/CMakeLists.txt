# found variable?
#include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test-drishti-eye test-drishti-eye.cpp test-EyeModelEstimator.cpp)
target_link_libraries(test-drishti-eye drishtisdk ${OpenCV_LIBS} GTest::main)

enable_testing()

if(MSVC)
  add_test(NAME DrishtiEyeTest COMMAND test-drishti-eye
    "${assets_dir}/drishti_eye_full_npd_eix.txt"
    "${assets_dir}/images/2318-eye.png"
    "${assets_dir}/images/2318-eye.json"
    "1"
    )
else()
  add_test(NAME DrishtiEyeTest COMMAND test-drishti-eye
    "${assets_dir}/drishti_eye_full_npd_eix.pba.z"
    "${assets_dir}/images/2318-eye.png"
    "${assets_dir}/images/2318-eye.json"
    "0"
    )
endif()

set_property(TARGET test-drishti-eye PROPERTY FOLDER "app/tests")
