# Copyright (c) 2015, Ruslan Baratov, David Hirvonen
# All rights reserved.

# Currently uses QT for windows-less context (GPGPU stuff),
# but should also tests CPU based ACF code in isolation.

set(DRISHTI_ACF_TEST_NAME DrishtiACFTest)
set(DRISHTI_ACF_TEST_APP test-drishti-acf)

if(DRISHTI_BUILD_QT AND DRISHTI_BUILD_OGLES_GPGPU)
  set(DRISHTI_ACF_DO_GPU 1)
else()
  set(DRISHTI_ACF_DO_GPU 0)
endif()

set(SOURCES
  test-Detector.cpp
  test-drishti-acf.cpp
  )

if(DRISHTI_ACF_DO_GPU)
  list(APPEND SOURCES QGLContext.h QGLContext.cpp)
endif()

add_executable(${DRISHTI_ACF_TEST_APP} ${SOURCES})

target_link_libraries(${DRISHTI_ACF_TEST_APP}
  ${OpenCV_LIBS}
  GTest::gtest
  drishtisdk
  ${DRISHTI_OPENGL_LIBS}
  )

if(DRISHTI_ACF_DO_GPU)
  find_package(Qt5PrintSupport REQUIRED)  
  target_link_libraries(${DRISHTI_ACF_TEST_APP} ${OGLES_GPGPU_LIB} Qt5::Widgets Qt5::PrintSupport)
  target_compile_definitions(${DRISHTI_ACF_TEST_APP} PUBLIC DRISHTI_ACF_DO_GPU=${DRISHTI_ACF_DO_GPU})
endif()

set_property(TARGET ${DRISHTI_ACF_TEST_APP} PROPERTY FOLDER "app/tests")

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

enable_testing()

if(ANDROID)
  
  # Extract filenames:
  get_filename_component(DRISHTI_TEST_FACE_IMAGE_COLOR_ "${DRISHTI_TEST_FACE_IMAGE_COLOR}" NAME)
  get_filename_component(DRISHTI_ACF_FACE_MODEL_ "${DRISHTI_ACF_FACE_MODEL}" NAME) 

  # Will set ${DRISHTI_ACF_TEST_NAME}_INSTALL_PATH
  drishti_android_add_test(NAME ${DRISHTI_ACF_TEST_NAME}
    COMMAND ${DRISHTI_ACF_TEST_APP}
    "${DRISHTI_TEST_FACE_IMAGE_COLOR_}"
    "${DRISHTI_TEST_FACE_IMAGE_COLOR_}" #= legacy
    "${DRISHTI_ACF_FACE_MODEL_}"
    "." # output to install path (current working directory)
    DEPENDENCIES
    "${DRISHTI_TEST_FACE_IMAGE_COLOR}"
    "${DRISHTI_ACF_FACE_MODEL}"    
    )
  
else()

  add_test(NAME ${DRISHTI_ACF_TEST_NAME} COMMAND ${DRISHTI_ACF_TEST_APP}
    "${DRISHTI_TEST_FACE_IMAGE_COLOR}"
    "${DRISHTI_TEST_FACE_IMAGE_COLOR}"
    "${DRISHTI_ACF_FACE_MODEL}"
    "${CMAKE_CURRENT_BINARY_DIR}" # Used as portable temp folder
    )

endif()

