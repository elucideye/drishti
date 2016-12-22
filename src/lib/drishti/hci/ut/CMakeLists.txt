# found variable?
#include_directories(${GTEST_INCLUDE_DIRS})

set(DRISHTI_HCI_TEST_NAME DrishtiHciTest)
set(DRISHTI_HCI_TEST_APP test-drishti-hci)

add_executable(${DRISHTI_HCI_TEST_APP} test-drishti-hci.cpp)
set_property(TARGET ${DRISHTI_HCI_TEST_APP} PROPERTY FOLDER "app/tests")
target_link_libraries(${DRISHTI_HCI_TEST_APP}
  drishtisdk
  ${OpenCV_LIBS}
  GTest::gtest
  ${DRISHTI_OPENGL_LIBS}
  )

# OS X Fix: "_CVPixelBufferLockBaseAddress", referenced from: (etc)
if(APPLE)
  if(IOS)
    target_link_libraries(${DRISHTI_HCI_TEST_APP}
      "-framework CoreFoundation"
      "-framework Foundation"  # _NSLog
      )
  else()
    target_link_libraries(${DRISHTI_HCI_TEST_APP}
      "-framework ImageIO"
      "-framework CoreVideo"
      )
  endif()
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

  # Extract filenames:
  get_filename_component(DRISHTI_ACF_FACE_MODEL_ "${DRISHTI_ACF_FACE_MODEL}" NAME)
  get_filename_component(DRISHTI_FACE_LANDMARKER_ "${DRISHTI_FACE_LANDMARKER}" NAME)
  get_filename_component(DRISHTI_EYE_MODEL_ "${DRISHTI_EYE_MODEL}" NAME)
  get_filename_component(DRISHTI_MEAN_FACE_LANDMARKS_ "${DRISHTI_MEAN_FACE_LANDMARKS}" NAME)

  # Will set ${DRISHTI_HCI_TEST_NAME}_INSTALL_PATH
  drishti_android_add_test(NAME ${DRISHTI_HCI_TEST_NAME}
    COMMAND ${DRISHTI_HCI_TEST_APP}
    "${DRISHTI_ACF_FACE_MODEL_}"
    "${DRISHTI_MEAN_FACE_LANDMARKS_}"
    "${DRISHTI_FACE_LANDMARKER_}"
    "${DRISHTI_EYE_MODEL_}"
    DEPENDENCIES
    "${DRISHTI_ACF_FACE_MODEL}"
    "${DRISHTI_FACE_LANDMARKER}"
    "${DRISHTI_EYE_MODEL}"
    "${DRISHTI_MEAN_FACE_LANDMARKS}"    
    )

else()

  add_test(NAME ${DRISHTI_HCI_TEST_NAME} COMMAND ${DRISHTI_HCI_TEST_APP}
    "${DRISHTI_ACF_FACE_MODEL}"
    "${DRISHTI_MEAN_FACE_LANDMARKS}"
    "${DRISHTI_FACE_LANDMARKER}"
    "${DRISHTI_EYE_MODEL}"
    )

endif()


