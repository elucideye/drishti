# Copyright (c) 2015, Ruslan Baratov, David Hirvonen
# All rights reserved.

# Currently uses QT for windows-less context (GPGPU stuff),
# but should also tests CPU based ACF code in isolation.

set(test_name DrishtiACFTest)
set(test_app test-drishti-acf)

# The GPU tests currently require QT for the off screen GL context
# and this won't be installed with the simple shared library
# dependency walker
# https://github.com/elucideye/drishti/issues/259
if(DRISHTI_BUILD_QT AND DRISHTI_BUILD_OGLES_GPGPU AND NOT ANDROID)
  set(DRISHTI_ACF_DO_GPU 1)
else()
  set(DRISHTI_ACF_DO_GPU 0)
endif()

set(SOURCES
  test-Detector.cpp
  test-drishti-acf.cpp
  )

### PRE_BUILD GPU ADDITIONS ###
if(DRISHTI_ACF_DO_GPU)
  list(APPEND SOURCES ${DRISHTI_QTPLUS_HDRS} ${DRISHTI_QTPLUS_SRCS})
endif()

add_executable(${test_app} ${SOURCES})
set_target_properties(${test_app} PROPERTIES FOLDER "app/tests" INTERPROCEDURAL_OPTIMIZATION ${DRISHTI_EXE_IPO})
target_link_libraries(${test_app} PUBLIC
  ${OpenCV_LIBS}
  GTest::gtest
  drishtisdk
  ${DRISHTI_OPENGL_LIBS}
  )

drishti_bool_to_int(DRISHTI_BUILD_QT build_qt)
target_compile_definitions(${test_app} PUBLIC DRISHTI_BUILD_QT=${build_qt})

### POST_BUILD GPU ADDITIONS ###
if(DRISHTI_ACF_DO_GPU)
  find_package(Qt5PrintSupport REQUIRED)
  target_link_libraries(${test_app} PUBLIC ${OGLES_GPGPU_LIB} Qt5::Widgets Qt5::PrintSupport)
  target_compile_definitions(${test_app} PUBLIC DRISHTI_ACF_DO_GPU=${DRISHTI_ACF_DO_GPU})
endif()

##
## GTest + CTest
##

include(drishti_set_unit_test_assets)
drishti_set_unit_test_assets(
    DRISHTI_ACF_FACE_MODEL
    DRISHTI_FACE_LANDMARKER
    DRISHTI_EYE_MODEL
    DRISHTI_MEAN_FACE_LANDMARKS
    )

set(DRISHTI_TEST_FACE_IMAGE_COLOR "${assets_dir}/images/lena512color.png")

if(NOT EXISTS "${DRISHTI_TEST_FACE_IMAGE_COLOR}")
  message(FATAL_ERROR "Failed to find test face image: ${DRISHTI_TEST_FACE_IMAGE_COLOR}")
endif()

drishti_add_test(
  NAME ${test_name}
  COMMAND ${test_app}
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_TEST_FACE_IMAGE_COLOR}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_TEST_FACE_IMAGE_COLOR}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_ACF_FACE_MODEL}>"
  "." # Used working directory as portable temp folder
  )

