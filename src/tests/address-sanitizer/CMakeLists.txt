add_executable(test-drishti-asan test-drishti-asan.cpp)
target_link_libraries(test-drishti-asan PUBLIC GTest::main)
set_target_properties(test-drishti-asan PROPERTIES FOLDER "app/tests" INTERPROCEDURAL_OPTIMIZATION ${DRISHTI_EXE_IPO})

enable_testing()

drishti_add_test(NAME DrishtiAsanTest COMMAND test-drishti-asan)

# Run with: `export ASAN_OPTIONS=abort_on_error=0` on clang for _exit(1) rather than abort()
# http://stackoverflow.com/q/33693486
set_tests_properties(DrishtiAsanTest
  PROPERTIES
  WILL_FAIL TRUE
  ENVIRONMENT "ASAN_OPTIONS=abort_on_error=0"
  )
