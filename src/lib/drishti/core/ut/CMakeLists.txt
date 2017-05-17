set(test_name DrishtiCoreTest)
set(test_app test-drishti-core)

add_executable(${test_app} test-convert.cpp test-drishti-core.cpp)
target_link_libraries(${test_app} PUBLIC drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_target_properties(${test_app} PROPERTIES FOLDER "app/tests" INTERPROCEDURAL_OPTIMIZATION ${DRISHTI_EXE_IPO})

drishti_add_test(
    NAME ${test_name}
    COMMAND ${test_app}
    )
