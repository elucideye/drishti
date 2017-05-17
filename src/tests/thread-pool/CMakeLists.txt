#### test-thread-pool ####
add_executable(test-thread-pool test-thread-pool.cpp)
target_link_libraries(test-thread-pool)
install(TARGETS test-thread-pool DESTINATION bin)
set_target_properties(test-thread-pool PROPERTIES FOLDER "app/tests" INTERPROCEDURAL_OPTIMIZATION ${DRISHTI_EXE_IPO})

