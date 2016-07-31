#### test-thread-pool ####
add_executable(test-thread-pool test-thread-pool.cpp)
target_link_libraries(test-thread-pool)
install(TARGETS test-thread-pool DESTINATION bin)
set_property(TARGET test-thread-pool PROPERTY FOLDER "app/tests")

