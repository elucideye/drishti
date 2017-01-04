set(test_name DrishtiGeometryTest)
set(test_app test-drishti-geometry)
if(IOS)
  set(test_app_type "MACOSX_BUNDLE")
endif()

add_executable("${test_app}" ${test_app_type} test-drishti-geometry.cpp)
target_link_libraries("${test_app}" drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_property(TARGET "${test_app}" PROPERTY FOLDER "app/tests")

enable_testing()

if(ANDROID)
  drishti_android_add_test(
    NAME "${test_name}"
    COMMAND "${test_app}" 
    CLEAN
    )
elseif(IOS)

  set(test_bundle_id "com.example.tests.drishti.geometry") 
  set_target_properties(
    "${test_app}"
    PROPERTIES
    MACOSX_BUNDLE_INFO_PLIST
    "${CMAKE_SOURCE_DIR}/cmake/templates/Info.plist.in"    
    XCODE_ATTRIBUTE_BUNDLE_IDENTIFIER    
    "${test_bundle_id}"
    XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY "1,2" # iPhone/iPad
    )
  
  drishti_ios_add_test(
    NAME ${test_name}
    COMMAND ${test_app}
    BUNDLE_ID "${test_bundle_id}"
    CLEAN
    )
else()  
  add_test(NAME ${test_name} COMMAND "${test_app}")
endif()

#  https://cmake.org/pipermail/cmake/2016-March/063115.html