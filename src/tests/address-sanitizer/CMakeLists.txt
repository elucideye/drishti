add_executable(test-drishti-asan test-drishti-asan.cpp)
target_link_libraries(test-drishti-asan GTest::main)

enable_testing()

add_test(NAME DrishtAsanTest COMMAND test-drishti-asan)

set_property(TARGET test-drishti-asan PROPERTY FOLDER "app/tests")
