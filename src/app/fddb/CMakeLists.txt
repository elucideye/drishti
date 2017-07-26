set(test_app drishti-fddb)
add_executable(${test_app} fddbcrop.cpp FDDB.h FDDB.cpp)
target_link_libraries(${test_app} ${DRISHTI_SDK_BOOST_LIBS} ${OpenCV_LIBS} cxxopts::cxxopts
  Boost::system
  Boost::filesystem      
  )
target_compile_definitions(${test_app} PUBLIC _USE_MATH_DEFINES)
target_include_directories(${test_app} PUBLIC "$<BUILD_INTERFACE:${DRISHTI_INCLUDE_DIRECTORIES}>")

if(DRISHTI_USE_IMSHOW)
  target_compile_definitions(${test_app} PUBLIC DRISHTI_USE_IMSHOW=1)
  target_link_libraries(${test_app} imshow::imshow)
endif()

set_property(TARGET ${test_app} PROPERTY FOLDER "app/console")
install(TARGETS ${test_app} DESTINATION bin)
