set(test_name NlohmannJsonTest)
set(test_app test-nlohmann-json)

hunter_add_package(nlohmann_json)
find_package(nlohmann_json CONFIG REQUIRED)

add_executable(${test_app} test-json.cpp)
target_link_libraries(${test_app} PUBLIC nlohmann_json GTest::gtest)
target_include_directories(${test_app} PUBLIC "$<BUILD_INTERFACE:${DRISHTI_INCLUDE_DIRECTORIES}>")
set_property(TARGET ${test_app} PROPERTY FOLDER "app/tests")

gauze_add_test(
  NAME ${test_name}
  COMMAND ${test_app}
  "$<GAUZE_RESOURCE_FILE:${CMAKE_CURRENT_SOURCE_DIR}/sample.json>"
  )
