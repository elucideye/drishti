# Copyright (c) 2015-2017, Ruslan Baratov, David Hirvonen
# All rights reserved.

set(test_name DrishtiACFTest)
set(test_app test-drishti-acf)

set(drishti_acf_ut_srcs test-Detector.cpp test-drishti-acf.cpp)

add_executable(${test_app} ${drishti_acf_ut_srcs})
set_property(TARGET ${test_app} PROPERTY FOLDER "app/tests")
target_link_libraries(${test_app} PUBLIC
  ${OpenCV_LIBS}
  GTest::gtest
  drishtisdk)

if ((TARGET drishti_gltest) AND DRISHTI_BUILD_OGLES_GPGPU)
  target_link_libraries(${test_app} PUBLIC drishti_gltest)
  target_compile_definitions(${test_app} PUBLIC DRISHTI_ACF_DO_GPU=1)
endif()

# GTest + CTest
include(drishti_set_unit_test_assets)
drishti_set_unit_test_assets(
    DRISHTI_ACF_FACE_MODEL
    DRISHTI_FACE_LANDMARKER
    DRISHTI_EYE_MODEL
    DRISHTI_MEAN_FACE_LANDMARKS)

set(DRISHTI_TEST_FACE_IMAGE_COLOR "${assets_dir}/images/lena512color.png")

if(NOT EXISTS "${DRISHTI_TEST_FACE_IMAGE_COLOR}")
  message(FATAL_ERROR "Failed to find test face image: ${DRISHTI_TEST_FACE_IMAGE_COLOR}")
endif()

gauze_add_test(
  NAME ${test_name}
  COMMAND ${test_app}
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_TEST_FACE_IMAGE_COLOR}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_TEST_FACE_IMAGE_COLOR}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_ACF_FACE_MODEL}>"
  "." # Used working directory as portable temp folder
  )

