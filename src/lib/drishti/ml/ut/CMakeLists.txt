# found variable?
#include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test-drishti-ml test-drishti-ml.cpp test-RTEShapeEstimator.cpp)
target_link_libraries(test-drishti-ml drishtisdk ${OpenCV_LIBS} GTest::gtest)

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
set(DRISHTI_TEST_FACE_MODEL "${assets_dir}/images/lena512color.json")

if(MSVC)
  add_test(NAME DrishtiMlTest COMMAND test-drishti-ml
    "${assets_dir}/drishti_face_inner.txt"
    "${DRISHTI_TEST_FACE}"
    "${DRISHTI_TEST_FACE_MODEL}"
    "${CMAKE_CURRENT_BINARY_DIR}"
    "1"
    )
else()
  add_test(NAME DrishtMlTest COMMAND test-drishti-ml
    "${DRISHTI_FACE_LANDMARKER}"
    "${DRISHTI_TEST_FACE}"    
    "${DRISHTI_TEST_FACE_MODEL}"
    "${CMAKE_CURRENT_BINARY_DIR}"
    "0"
    )
endif()

######

#add_test(NAME DrishtiMlTest COMMAND test-drishti-ml)

set_property(TARGET test-drishti-ml PROPERTY FOLDER "app/tests")
