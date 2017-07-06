set(test_name DrishtiFaceTest)
set(test_app test-drishti-face)

add_executable(${test_app} test-drishti-face.cpp)
target_link_libraries(${test_app} PUBLIC drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_property(TARGET ${test_app} PROPERTY FOLDER "app/tests")

# OS X Fix: "_CVPixelBufferLockBaseAddress", referenced from: (etc)
if(APPLE AND NOT IOS)
  target_link_libraries(${test_app} PUBLIC
    "-framework ImageIO"
    "-framework CoreVideo"
    )
endif()

gauze_add_test(
  NAME ${test_name}
  COMMAND ${test_app}
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_ASSETS_FACE_DETECTOR}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_ASSETS_FACE_DETECTOR_MEAN}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_ASSETS_FACE_LANDMARK_REGRESSOR}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_ASSETS_EYE_MODEL_REGRESSOR}>"
  )
