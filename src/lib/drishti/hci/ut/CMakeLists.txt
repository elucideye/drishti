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

include(drishti_set_unit_test_assets)
drishti_set_unit_test_assets(
  DRISHTI_ACF_FACE_MODEL
  DRISHTI_FACE_LANDMARKER
  DRISHTI_EYE_MODEL
  DRISHTI_MEAN_FACE_LANDMARKS)

set(DRISHTI_TEST_FACE_IMAGE_COLOR "${assets_dir}/images/lena512color.png")

drishti_add_test(NAME ${test_name} COMMAND ${test_app}
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_ACF_FACE_MODEL}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_MEAN_FACE_LANDMARKS}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_FACE_LANDMARKER}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_EYE_MODEL}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_TEST_FACE_IMAGE_COLOR}>")
