add_executable(test-drishti-asan test-drishti-asan.cpp)
target_link_libraries(test-drishti-asan PUBLIC GTest::main)
set_property(TARGET test-drishti-asan PROPERTY FOLDER "app/tests")

enable_testing()

gauze_add_test(NAME DrishtiAsanTest COMMAND test-drishti-asan)

# Run with: `export ASAN_OPTIONS=abort_on_error=0` on clang for _exit(1) rather than abort()
# http://stackoverflow.com/q/33693486
set_tests_properties(DrishtiAsanTest
  PROPERTIES
  WILL_FAIL TRUE
  ENVIRONMENT "ASAN_OPTIONS=abort_on_error=0"
  )
