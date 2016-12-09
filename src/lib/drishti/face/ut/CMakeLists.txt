# found variable?
#include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test-drishti-face test-drishti-face.cpp)
target_link_libraries(test-drishti-face drishtisdk ${OpenCV_LIBS} GTest::gtest)

# OS X Fix: "_CVPixelBufferLockBaseAddress", referenced from: (etc)
if(APPLE AND NOT IOS)
  target_link_libraries(test-drishti-face
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

add_test(NAME DrishtiFaceTest COMMAND test-drishti-face
  "${DRISHTI_ACF_FACE_MODEL}"
  "${DRISHTI_MEAN_FACE_LANDMARKS}"
  "${DRISHTI_FACE_LANDMARKER}"
  "${DRISHTI_EYE_MODEL}"
  )

set_property(TARGET test-drishti-face PROPERTY FOLDER "app/tests")
