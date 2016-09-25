add_executable(pba2txt pba2txt.cpp)
target_link_libraries(pba2txt drishtisdk)
set_property(TARGET pba2txt PROPERTY FOLDER "app/console")
install(TARGETS pba2txt DESTINATION bin)
