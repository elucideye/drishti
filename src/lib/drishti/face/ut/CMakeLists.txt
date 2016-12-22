# found variable?
#include_directories(${GTEST_INCLUDE_DIRS})

set(DRISHTI_FACE_TEST_NAME DrishtiFaceTest)
set(DRISHTI_FACE_TEST_APP test-drishti-face)

add_executable(${DRISHTI_FACE_TEST_APP} test-drishti-face.cpp)
target_link_libraries(${DRISHTI_FACE_TEST_APP} drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_property(TARGET ${DRISHTI_FACE_TEST_APP} PROPERTY FOLDER "app/tests")

# OS X Fix: "_CVPixelBufferLockBaseAddress", referenced from: (etc)
if(APPLE AND NOT IOS)
  target_link_libraries(${DRISHTI_FACE_TEST_APP}
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

enable_testing()

if(ANDROID) 

  set(DRISHTI_FACE_TEST_DATA
    "${DRISHTI_ACF_FACE_MODEL}"
    "${DRISHTI_FACE_LANDMARKER}"
    "${DRISHTI_EYE_MODEL}"
    "${DRISHTI_MEAN_FACE_LANDMARKS}"
    )

  # Extract filenames:
  get_filename_component(DRISHTI_ACF_FACE_MODEL_ "${DRISHTI_ACF_FACE_MODEL}" NAME)
  get_filename_component(DRISHTI_FACE_LANDMARKER_ "${DRISHTI_FACE_LANDMARKER}" NAME)
  get_filename_component(DRISHTI_EYE_MODEL_ "${DRISHTI_EYE_MODEL}" NAME)
  get_filename_component(DRISHTI_MEAN_FACE_LANDMARKS_ "${DRISHTI_MEAN_FACE_LANDMARKS}" NAME)

  # Will set ${DRISHTI_FACE_TEST_NAME}_INSTALL_PATH
  drishti_android_add_test(NAME ${DRISHTI_FACE_TEST_NAME} COMMAND ${DRISHTI_FACE_TEST_APP}
    "${DRISHTI_ACF_FACE_MODEL_}"
    "${DRISHTI_MEAN_FACE_LANDMARKS_}"
    "${DRISHTI_FACE_LANDMARKER_}"
    "${DRISHTI_EYE_MODEL_}"
    )

  drishti_android_copy_files(
    "${DRISHTI_FACE_TEST_APP}"
    "${DRISHTI_FACE_TEST_DATA}"
    "${${DRISHTI_FACE_TEST_NAME}_INSTALL_PATH}"
    )
  
else()

  add_test(NAME ${DRISHTI_FACE_TEST_NAME} COMMAND ${DRISHTI_FACE_TEST_APP}
    "${DRISHTI_ACF_FACE_MODEL}"
    "${DRISHTI_MEAN_FACE_LANDMARKS}"
    "${DRISHTI_FACE_LANDMARKER}"
    "${DRISHTI_EYE_MODEL}"
    )

endif()

