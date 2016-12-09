# found variable?
#include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test-drishti-hci test-drishti-hci.cpp)
target_link_libraries(test-drishti-hci
  drishtisdk
  ${OpenCV_LIBS}
  GTest::gtest
  ${DRISHTI_OPENGL_LIBS}
  )

# OS X Fix: "_CVPixelBufferLockBaseAddress", referenced from: (etc)
if(APPLE)
  if(IOS)
    target_link_libraries(test-drishti-hci
      "-framework CoreFoundation"
      "-framework Foundation"  # _NSLog
      )
  else()
    target_link_libraries(test-drishti-hci
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

add_test(NAME DrishtiHciTest COMMAND test-drishti-hci
  "${DRISHTI_ACF_FACE_MODEL}"
  "${DRISHTI_MEAN_FACE_LANDMARKS}"
  "${DRISHTI_FACE_LANDMARKER}"
  "${DRISHTI_EYE_MODEL}"
  )

set_property(TARGET test-drishti-hci PROPERTY FOLDER "app/tests")
