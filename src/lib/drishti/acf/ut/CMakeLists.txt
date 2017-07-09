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

if ((${DRISHTI_DO_GPU_TESTING}) AND DRISHTI_BUILD_OGLES_GPGPU)
  target_link_libraries(${test_app} PUBLIC aglet::aglet)
  target_compile_definitions(${test_app} PUBLIC DRISHTI_ACF_DO_GPU=1)
endif()

# GTest + CTest
gauze_add_test(
  NAME ${test_name}
  COMMAND ${test_app}
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_FACES_FACE_IMAGE}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_FACES_FACE_IMAGE}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_ASSETS_FACE_DETECTOR}>"
  "." # Used working directory as portable temp folder
  )

