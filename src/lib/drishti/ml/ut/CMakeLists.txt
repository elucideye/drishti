set(test_name DrishtiMlTest)
set(test_app test-drishti-ml)

add_executable(${test_app} test-drishti-ml.cpp test-RTEShapeEstimator.cpp)
target_link_libraries(${test_app} PUBLIC drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_property(TARGET ${test_app} PROPERTY FOLDER "app/tests")

gauze_add_test(
  NAME ${test_name}
  COMMAND ${test_app}
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_ASSETS_FACE_LANDMARK_REGRESSOR}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_FACES_FACE_IMAGE}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_FACES_FACE_IMAGE}>" # TODO _MODEL
  "."
  "0"
  )
