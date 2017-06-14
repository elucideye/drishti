set(test_name DrishtiGeometryTest)
set(test_app test-drishti-geometry)

add_executable(${test_app} test-drishti-geometry.cpp)
target_link_libraries(${test_app} PUBLIC drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_property(TARGET ${test_app} PROPERTY FOLDER "app/tests")

gauze_add_test(
  NAME ${test_name}
  COMMAND ${test_app}
  )
