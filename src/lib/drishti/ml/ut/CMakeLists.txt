set(test_name DrishtiMlTest)
set(test_app test-drishti-ml)

add_executable(${test_app} test-drishti-ml.cpp test-RTEShapeEstimator.cpp)
target_link_libraries(${test_app} PUBLIC drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_property(TARGET ${test_app} PROPERTY FOLDER "app/tests")

include(drishti_set_unit_test_assets)
drishti_set_unit_test_assets(
  DRISHTI_ACF_FACE_MODEL
  DRISHTI_FACE_LANDMARKER
  DRISHTI_EYE_MODEL
  DRISHTI_MEAN_FACE_LANDMARKS
  )

set(DRISHTI_TEST_FACE "${assets_dir}/images/lena512color.png")
set(DRISHTI_TEST_FACE_MODEL "${assets_dir}/images/lena512color.json") # missing

gauze_add_test(
  NAME ${test_name}
  COMMAND ${test_app}
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_FACE_LANDMARKER}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_TEST_FACE}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_TEST_FACE}>" # TODO _MODEL
  "."
  "0"
  )
