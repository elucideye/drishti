# found variable?
#include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test-drishti-ml test-drishti-ml.cpp)
target_link_libraries(test-drishti-ml drishtisdk ${OpenCV_LIBS} GTest::gtest)

enable_testing()

add_test(NAME DrishtiMlTest COMMAND test-drishti-ml)

set_property(TARGET test-drishti-ml PROPERTY FOLDER "app/tests")
