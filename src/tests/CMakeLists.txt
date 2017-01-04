if(DRISHTI_USE_THREAD_POOL_CPP)
#  add_subdirectory(thread-pool)
endif()

#add_subdirectory(serialize)

# Seems to be failing in cache only builds with hunter, even after 17.2 "cacheable" update:
#add_subdirectory(json)

if(DRISHTI_BUILD_ASAN_TEST)
  add_subdirectory(address-sanitizer)
endif()