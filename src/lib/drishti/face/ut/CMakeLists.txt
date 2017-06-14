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

include(drishti_set_unit_test_assets)
drishti_set_unit_test_assets(
  DRISHTI_ACF_FACE_MODEL
  DRISHTI_FACE_LANDMARKER
  DRISHTI_EYE_MODEL
  DRISHTI_MEAN_FACE_LANDMARKS
  )

gauze_add_test(
  NAME ${test_name}
  COMMAND ${test_app}
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_ACF_FACE_MODEL}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_MEAN_FACE_LANDMARKS}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_FACE_LANDMARKER}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_EYE_MODEL}>"
  )
