########################
### drishti-eye-test ###
########################

add_executable(drishti-eye-test drishti-eye-test.cpp)
target_link_libraries(drishti-eye-test PUBLIC ${base_deps})
if(DRISHTI_TEST_BUILD_TESTS)
  target_compile_definitions(drishti-eye-test PUBLIC DRISHTI_TEST_BUILD_TESTS=1)
endif()
if(DRISHTI_HUNTER_TEST_ADD_TO_STRING)
  target_compile_definitions(drishti-eye-test PUBLIC DRISHTI_HUNTER_TEST_ADD_TO_STRING=1)
endif()
install(TARGETS drishti-eye-test DESTINATION bin)
