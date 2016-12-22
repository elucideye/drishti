# found variable?
#include_directories(${GTEST_INCLUDE_DIRS})

set(DRISHTI_SDK_TEST_NAME DrishtiDrishtiEyeTest)
set(DRISHTI_SDK_TEST_APP test-drishti-drishti-eye)

add_executable(${DRISHTI_SDK_TEST_APP} test-EyeSegmenter.cpp)
set_property(TARGET ${DRISHTI_SDK_TEST_APP} PROPERTY FOLDER "app/tests")
target_link_libraries(${DRISHTI_SDK_TEST_APP} drishti
  ${OpenCV_LIBS}
  GTest::gtest
  drishtisdk # include drishtisdk for drishti::eye::EyeModel serialization
  )

add_executable(test-drishti-drishti-face test-FaceTracker.cpp)
set_property(TARGET test-drishti-drishti-face PROPERTY FOLDER "app/tests")
target_link_libraries(test-drishti-drishti-face drishti
  ${OpenCV_LIBS}
  GTest::gtest
  drishtisdk # include drishtisdk for drishti::eye::EyeModel serialization
  )

include(drishti_set_unit_test_assets)
drishti_set_unit_test_assets(
    DRISHTI_ACF_FACE_MODEL
    DRISHTI_FACE_LANDMARKER
    DRISHTI_EYE_MODEL
    DRISHTI_MEAN_FACE_LANDMARKS
    )

#######################################################################
message("DRISHTI_EYE_MODEL : ${DRISHTI_EYE_MODEL}")

# Legacy
if(MSVC)
  set(DRISHTI_TXT_ARCHIVE 1)
else()
  set(DRISHTI_TXT_ARCHIVE 0)
endif()  

enable_testing()

set(DRISHTI_TEST_EYE_IMAGE "${assets_dir}/images/2318-eye.png")
set(DRISHTI_TEST_EYE_JSON "${assets_dir}/images/2318-eye.json")    

if(ANDROID)


else()

  add_test(NAME ${DRISHTI_SDK_TEST_NAME} COMMAND ${DRISHTI_SDK_TEST_APP}
    "${DRISHTI_EYE_MODEL}"
    "${DRISHTI_TEST_EYE_IMAGE}"
    "${DRISHTI_TEST_EYE_JSON}"
    "${DRISHTI_TXT_ARCHIVE}"
    )

  if(WIN32)
    set_property(
      TEST ${DRISHTI_SDK_TEST_NAME}
      PROPERTY
      ENVIRONMENT
      "PATH=$<TARGET_FILE_DIR:drishti>;$ENV{PATH}"
      )
  endif()

endif()
