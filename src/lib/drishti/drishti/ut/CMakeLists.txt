set(test_name DrishtiDrishtiEyeTest)
set(test_app test-drishti-drishti-eye)

# Use static lib to avoid dynamic frameworks in internal tests
if(IOS)
  set(drishti_sdk_lib drishti_static)
else()
  set(drishti_sdk_lib drishti)
endif()

add_executable(${test_app} test-EyeSegmenter.cpp)
set_property(TARGET ${test_app} PROPERTY FOLDER "app/tests")
target_link_libraries(${test_app} PUBLIC ${drishti_sdk_lib}
  ${OpenCV_LIBS}
  GTest::gtest
  drishtisdk # include drishtisdk for drishti::eye::EyeModel serialization
  )

include(drishti_set_unit_test_assets)
drishti_set_unit_test_assets(
    DRISHTI_ACF_FACE_MODEL
    DRISHTI_FACE_LANDMARKER
    DRISHTI_EYE_MODEL
    DRISHTI_MEAN_FACE_LANDMARKS
    )

#######################################################################
message("DRISHTI_EYE_MODEL : ${DRISHTI_EYE_MODEL}")

enable_testing()

set(DRISHTI_TEST_EYE_IMAGE "${assets_dir}/images/2318-eye.png")
set(DRISHTI_TEST_EYE_JSON "${assets_dir}/images/2318-eye.json")

drishti_add_test(
  NAME ${test_name}
  COMMAND ${test_app}
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_EYE_MODEL}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_TEST_EYE_IMAGE}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_TEST_EYE_JSON}>"
  "0"
  )

if(WIN32)
  set_property(
    TEST ${test_name}
    PROPERTY
    ENVIRONMENT
    "PATH=$<TARGET_FILE_DIR:drishti>;$ENV{PATH}"
    )
endif()
