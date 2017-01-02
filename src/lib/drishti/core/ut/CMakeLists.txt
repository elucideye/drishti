if(IOS)
  set(TEST_APP_TYPE "MACOSX_BUNDLE")
endif()

add_executable(test-drishti-core ${TEST_APP_TYPE} test-convert.cpp test-drishti-core.cpp)
target_link_libraries(test-drishti-core drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_property(TARGET test-drishti-core PROPERTY FOLDER "app/tests")

enable_testing()

if(ANDROID)
  drishti_android_add_test(
    NAME DrishtiCoreTest
    COMMAND test-drishti-core
    CLEAN
    )
elseif(IOS)

  set(DRISHTI_CORE_BUNDLE_IDENTIFIER "com.github.elucideye.drishti.core") 
  set_target_properties(
    test-drishti-core
    PROPERTIES
    MACOSX_BUNDLE_INFO_PLIST
    "${CMAKE_SOURCE_DIR}/cmake/templates/Info.plist.in"
    XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER
    "com.github.elucideye.drishti.core"
    XCODE_ATTRIBUTE_BUNDLE_IDENTIFIER
    "com.github.elucideye.drishti.core"
    XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY "1,2" # iPhone/iPad
    )

  set(DRISHTI_TEST_FACE "${assets_dir}/images/lena512color.png")
  drishti_ios_add_test(
    NAME DrishtiCoreTest
    COMMAND test-drishti-core
    BUNDLE_ID "${DRISHTI_CORE_BUNDLE_IDENTIFIER}"
    DEPENDENCIES "${DRISHTI_TEST_FACE}"
    CLEAN
    )
else()
  add_test(NAME DrishtiCoreTest COMMAND test-drishti-core)
endif()
