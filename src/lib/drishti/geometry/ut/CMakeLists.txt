if(IOS)
  set(TEST_APP_TYPE "MACOSX_BUNDLE")
endif()

add_executable(test-drishti-geometry ${TEST_APP_TYPE} test-drishti-geometry.cpp)
target_link_libraries(test-drishti-geometry drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_property(TARGET test-drishti-geometry PROPERTY FOLDER "app/tests")

enable_testing()

if(ANDROID)
  drishti_android_add_test(
    NAME DrishtiGeometryTest
    COMMAND test-drishti-geometry
    CLEAN
    )
elseif(IOS)
  drishti_ios_add_test(
    NAME DrishtiGeometryTest
    COMMAND test-drishti-geometry
    CLEAN
    )
else()  
  add_test(NAME DrishtiGeometryTest COMMAND test-drishti-geometry)
endif()

#  https://cmake.org/pipermail/cmake/2016-March/063115.html