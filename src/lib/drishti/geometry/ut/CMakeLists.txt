# found variable?
#include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test-drishti-geometry test-drishti-geometry.cpp)
target_link_libraries(test-drishti-geometry drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_property(TARGET test-drishti-geometry PROPERTY FOLDER "app/tests")

enable_testing()

if(ANDROID)
  android_add_test(NAME DrishtiGeometryTest COMMAND test-drishti-geometry)
else()
  add_test(NAME DrishtiGeometryTest COMMAND test-drishti-geometry)
endif()
