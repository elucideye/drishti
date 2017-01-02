if(IOS)
  set(TEST_APP_TYPE "MACOSX_BUNDLE")
endif()

set(DRISHTI_APP_NAME test-drishti-geometry)

add_executable("${DRISHTI_APP_NAME}" ${TEST_APP_TYPE} test-drishti-geometry.cpp)
target_link_libraries("${DRISHTI_APP_NAME}" drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_property(TARGET "${DRISHTI_APP_NAME}" PROPERTY FOLDER "app/tests")

enable_testing()

if(ANDROID)
  drishti_android_add_test(
    NAME DrishtiGeometryTest
    COMMAND "${DRISHTI_APP_NAME}"
    CLEAN
    )
elseif(IOS)

  set(DRISHTI_GEOMETRY_BUNDLE_IDENTIFIER "com.github.elucideye.drishti.geometry") 
  set_target_properties(
    "${DRISHTI_APP_NAME}"
    PROPERTIES
    MACOSX_BUNDLE_INFO_PLIST
    "${CMAKE_SOURCE_DIR}/cmake/templates/Info.plist.in"    
    XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER    
    "${DRISHTI_GEOMETRY_BUNDLE_IDENTIFIER}"
    XCODE_ATTRIBUTE_BUNDLE_IDENTIFIER    
    "${DRISHTI_GEOMETRY_BUNDLE_IDENTIFIER}"
    XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY "1,2" # iPhone/iPad
    )
  
  drishti_ios_add_test(
    NAME DrishtiGeometryTest
    COMMAND ${DRISHTI_APP_NAME}
    BUNDLE_ID "${DRISHTI_GEOMETRY_BUNDLE_IDENTIFIER}"
    CLEAN
    )
else()  
  add_test(NAME DrishtiGeometryTest COMMAND "${DRISHTI_APP_NAME}")
endif()

#  https://cmake.org/pipermail/cmake/2016-March/063115.html