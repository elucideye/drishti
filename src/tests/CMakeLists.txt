if(DRISHTI_USE_THREAD_POOL_CPP)
#  add_subdirectory(thread-pool)
endif()

add_subdirectory(flatbuffers)

if(DRISHTI_BUILD_ASAN_TEST)
  add_subdirectory(address-sanitizer)
endif()