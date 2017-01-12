set(test_name NlohmannJsonTest)
set(test_app test-nlohmann-json)

hunter_add_package(nlohmann-json)
find_package(nlohmann-json REQUIRED)

add_executable(${test_app} test-json.cpp)
target_link_libraries(${test_app} PUBLIC nlohmann-json::nlohmann-json)
set_property(TARGET ${test_app} PROPERTY FOLDER "app/tests")

drishti_add_test(
  NAME ${test_name}
  COMMAND ${test_app}
  "$<DRISHTI_RESOURCE_FILE:${CMAKE_CURRENT_SOURCE_DIR}/sample.json>"
  )
