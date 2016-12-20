# found variable?
#include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test-drishti-core test-convert.cpp test-drishti-core.cpp)
target_link_libraries(test-drishti-core drishtisdk ${OpenCV_LIBS} GTest::gtest)

enable_testing()

add_test(NAME DrishtiCoreTest COMMAND test-drishti-core)

set_property(TARGET test-drishti-core PROPERTY FOLDER "app/tests")

if(ANDROID)
  message("Add DrishtiCoreTestAndroid")
  include(AndroidApk) # android_add_test
  android_add_test(NAME DrishtiCoreTestAndroid COMMAND test-drishti-core)
endif()
