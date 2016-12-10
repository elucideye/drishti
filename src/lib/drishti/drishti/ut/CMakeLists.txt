# found variable?
#include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test-drishti-drishti test-EyeSegmenter.cpp test-drishti-drishti.cpp)
target_link_libraries(test-drishti-drishti drishti
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

# Legacy
if(MSVC)
  set(DRISHTI_TXT_ARCHIVE 1)
else()
  set(DRISHTI_TXT_ARCHIVE 0)
endif()  

enable_testing()

add_test(NAME DrishtiDrishtiTest COMMAND test-drishti-drishti
  "${DRISHTI_EYE_MODEL}"
  "${assets_dir}/images/2318-eye.png"
  "${assets_dir}/images/2318-eye.json"
  "${DRISHTI_TXT_ARCHIVE}"
  )

if(WIN32)
  set_property(
      TEST DrishtiDrishtiTest
      PROPERTY
      ENVIRONMENT
      "PATH=$<TARGET_FILE_DIR:drishti>;$ENV{PATH}"
  )
endif()

set_property(TARGET test-drishti-drishti PROPERTY FOLDER "app/tests")
