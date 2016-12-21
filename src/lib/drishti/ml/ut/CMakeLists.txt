# found variable?
#include_directories(${GTEST_INCLUDE_DIRS})

set(DRISHTI_ML_TEST_NAME DrishtiMlTest)
set(DRISHTI_ML_TEST_APP test-drishti-ml)

add_executable(${DRISHTI_ML_TEST_APP} test-drishti-ml.cpp test-RTEShapeEstimator.cpp)
target_link_libraries(${DRISHTI_ML_TEST_APP} drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_property(TARGET ${DRISHTI_ML_TEST_APP} PROPERTY FOLDER "app/tests")

include(drishti_set_unit_test_assets)
drishti_set_unit_test_assets(
    DRISHTI_ACF_FACE_MODEL
    DRISHTI_FACE_LANDMARKER
    DRISHTI_EYE_MODEL
    DRISHTI_MEAN_FACE_LANDMARKS
    )

enable_testing()

#####

set(DRISHTI_TEST_FACE "${assets_dir}/images/lena512color.png")
set(DRISHTI_TEST_FACE_MODEL "${assets_dir}/images/lena512color.json") # missing

if(ANDROID)

  if(HUNTER_ENABLED)
    hunter_add_package(Android-SDK)
    set(ADB_COMMAND "${ANDROID-SDK_ROOT}/android-sdk/platform-tools/adb")
  else()
    set(ADB_COMMAND "adb")
  endif()    

  set(ANDROID_APK_APP_DESTINATION "/data/local/tmp/AndroidApk")
  set(APP_DESTINATION_DIR "${ANDROID_APK_APP_DESTINATION}")
  set(APP_DESTINATION_DIR "${APP_DESTINATION_DIR}/${PROJECT_NAME}/AndroidTest")
  set(APP_DESTINATION_DIR "${APP_DESTINATION_DIR}/${DRISHTI_ML_TEST_NAME}")
  set(APP_DESTINATION "${APP_DESTINATION_DIR}/${DRISHTI_ML_TEST_APP}")

  
  set(DRISHTI_ML_TEST_DATA
    "${DRISHTI_FACE_LANDMARKER}"
    "${DRISHTI_TEST_FACE}"    
    "${DRISHTI_TEST_FACE_MODEL}"
    )
  
  foreach(local_file ${DRISHTI_ML_TEST_DATA})
    if(EXISTS "${local_file}")
      add_custom_command(
        TARGET ${DRISHTI_ML_TEST_APP}
        PRE_BUILD
        COMMAND ${ADB_COMMAND} push "${local_file}" "${APP_DESTINATION_DIR}"
        )
    endif()
  endforeach()

  get_filename_component(DRISHTI_FACE_LANDMARKER_ "${DRISHTI_FACE_LANDMARKER}" NAME)
  get_filename_component(DRISHTI_TEST_FACE_ "${DRISHTI_TEST_FACE}" NAME)
  get_filename_component(DRISHTI_TEST_FACE_MODEL_ "${DRISHTI_TEST_FACE_MODEL}" NAME)
  
  android_add_test(NAME ${DRISHTI_ML_TEST_NAME} COMMAND ${DRISHTI_ML_TEST_APP}
    "${APP_DESTINATION_DIR}/${DRISHTI_FACE_LANDMARKER_}"
    "${APP_DESTINATION_DIR}/${DRISHTI_TEST_FACE_}"    
    "${APP_DESTINATION_DIR}/${DRISHTI_TEST_FACE_MODEL_}"
    "${APP_DESTINATION_DIR}"
    "0"
    )
else()
  add_test(NAME ${DRISHTI_ML_TEST_NAME} COMMAND ${DRISHTI_ML_TEST_APP}
    "${DRISHTI_FACE_LANDMARKER}"
    "${DRISHTI_TEST_FACE}"    
    "${DRISHTI_TEST_FACE_MODEL}"
    "${CMAKE_CURRENT_BINARY_DIR}"
    "0"
    )
endif()
