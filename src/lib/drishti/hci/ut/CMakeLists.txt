set(test_name DrishtiHciTest)
set(test_app test-drishti-hci)

set(drishti_hci_ut_srcs
  test-drishti-hci.cpp
  test-hessian-cpu.cpp
  test-hessian-cpu.h
  test-FaceFinder.cpp
  FaceMonitorHCITest.h)

# http://stackoverflow.com/a/39966603
# http://stackoverflow.com/questions/32988266/unable-to-instantiate-the-uiapplication-subclass-instance-no-class-named-is
add_executable(${test_app} ${drishti_hci_ut_srcs})
set_property(TARGET ${test_app} PROPERTY FOLDER "app/tests")
target_link_libraries(${test_app} PUBLIC
  ${OpenCV_LIBS}
  GTest::gtest
  drishtisdk)

if((TARGET drishti_gltest) AND DRISHTI_BUILD_OGLES_GPGPU)
  target_link_libraries(${test_app} PUBLIC drishti_gltest)
  target_compile_definitions(${test_app} PUBLIC DRISHTI_HCI_DO_GPU=1)
endif()

gauze_add_test(NAME ${test_name} COMMAND ${test_app}
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_ASSETS_FACE_DETECTOR}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_ASSETS_FACE_DETECTOR_MEAN}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_ASSETS_FACE_LANDMARK_REGRESSOR}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_ASSETS_EYE_MODEL_REGRESSOR}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_FACES_FACE_IMAGE}>")
