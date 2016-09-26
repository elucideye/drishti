# found variable?
#include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test-drishti-geometry test-drishti-geometry.cpp)
target_link_libraries(test-drishti-geometry drishtisdk ${OpenCV_LIBS} GTest::main)

enable_testing()

add_test(NAME DrishtiGeometryTest COMMAND test-drishti-geometry)

set_property(TARGET test-drishti-geometry PROPERTY FOLDER "app/tests")
