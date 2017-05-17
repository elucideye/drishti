set(test_name DrishtiEyeTest)
set(test_app test-drishti-eye)

add_executable(${test_app} test-drishti-eye.cpp test-EyeModelEstimator.cpp)
target_link_libraries(${test_app} PUBLIC drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_target_properties(${test_app} PROPERTIES FOLDER "app/tests" INTERPROCEDURAL_OPTIMIZATION ${DRISHTI_EXE_IPO})

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

set(DRISHTI_TEST_EYE "${assets_dir}/images/2318-eye.png")
set(DRISHTI_TEST_EYE_MODEL_PRIVATE "${assets_dir}/images/2318-eye_private.json")
message("DRISHTI_TEST_EYE_MODEL_PRIVATE ${DRISHTI_TEST_EYE_MODEL_PRIVATE}")

drishti_add_test(
  NAME ${test_name}
  COMMAND ${test_app}
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_EYE_MODEL}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_TEST_EYE}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_TEST_EYE_MODEL_PRIVATE}>"
  "."
  "0" # legacy
  )
