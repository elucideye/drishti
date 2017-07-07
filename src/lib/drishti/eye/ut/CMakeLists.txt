set(test_name DrishtiEyeTest)
set(test_app test-drishti-eye)

add_executable(${test_app} test-drishti-eye.cpp test-EyeModelEstimator.cpp)
target_link_libraries(${test_app} PUBLIC drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_property(TARGET ${test_app} PROPERTY FOLDER "app/tests")

gauze_add_test(
  NAME ${test_name}
  COMMAND ${test_app}
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_ASSETS_EYE_MODEL_REGRESSOR}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_FACES_EYE_IMAGE}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_FACES_EYE_MODEL_PRIVATE}>"
  "."
  "0" # legacy
  )
