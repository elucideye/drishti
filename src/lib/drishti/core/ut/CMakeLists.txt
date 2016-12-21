# found variable?
#include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test-drishti-core test-convert.cpp test-drishti-core.cpp)
target_link_libraries(test-drishti-core drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_property(TARGET test-drishti-core PROPERTY FOLDER "app/tests")

enable_testing()

if(ANDROID)
  android_add_test(NAME DrishtiCoreTestAndroid COMMAND test-drishti-core)
else()
  add_test(NAME DrishtiCoreTest COMMAND test-drishti-core)
endif()
