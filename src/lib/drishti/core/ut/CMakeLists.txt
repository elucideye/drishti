set(DRISHTI_TEST_NAME DrishtiCoreTest)
set(DRISHTI_TEST_APP test-drishti-core)
if(IOS)
  set(TEST_APP_TYPE "MACOSX_BUNDLE")
endif()

add_executable(${DRISHTI_TEST_APP} ${TEST_APP_TYPE} test-convert.cpp test-drishti-core.cpp)
target_link_libraries(${DRISHTI_TEST_APP} drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_property(TARGET ${DRISHTI_TEST_APP} PROPERTY FOLDER "app/tests")
if(DRISHTI_HAVE_REGEX_H)
  target_compile_definitions(${DRISHTI_TEST_APP} PUBLIC DRISHTI_HAVE_REGEX_H)
endif()

enable_testing()

if(ANDROID)
  drishti_android_add_test(
    NAME "${DRISHTI_TEST_NAME}"
    COMMAND "${DRISHTI_TEST_APP}"
    CLEAN
    )
elseif(IOS)

  set(DRISHTI_TEST_BUNDLE_IDENTIFIER "com.elucideye.drishti.core") 
  set_target_properties(
    ${DRISHTI_TEST_APP}
    PROPERTIES
    MACOSX_BUNDLE_INFO_PLIST
    "${CMAKE_SOURCE_DIR}/cmake/templates/Info.plist.in"
    XCODE_ATTRIBUTE_BUNDLE_IDENTIFIER
    "${DRISHTI_TEST_BUNDLE_IDENTIFIER}"
    XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY "1,2" # iPhone/iPad
    )

  drishti_ios_add_test(
    NAME ${DRISHTI_TEST_NAME}
    COMMAND ${DRISHTI_TEST_APP}
    BUNDLE_ID "${DRISHTI_TEST_BUNDLE_IDENTIFIER}"
    CLEAN
    )
else()
  add_test(NAME ${DRISHTI_TEST_NAME} COMMAND ${DRISHTI_TEST_APP})
endif()
