add_executable(pba2txt pba2txt.cpp)
target_link_libraries(pba2txt drishtisdk)
set_target_properties(pba2txt PROPERTIES FOLDER "app/console" INTERPROCEDURAL_OPTIMIZATION ${DRISHTI_EXE_IPO})
install(TARGETS pba2txt DESTINATION bin)
