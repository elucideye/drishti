set(test_app drishti-facecrop)

add_executable(${test_app} facecrop.cpp
  FaceJitterer.h
  FaceJitterer.cpp
  FaceSpecification.h
  JitterParams.h
  JitterParams.cpp
  Pyramid.h
  Pyramid.cpp
  )

target_link_libraries(${test_app} drishtisdk cxxopts::cxxopts ${OpenCV_LIBS} drishti_landmarks)

if(DRISHTI_BUILD_EOS)
  target_link_libraries(${test_app} eos::eos)
  target_compile_definitions(${test_app} PUBLIC DRISHTI_BUILD_EOS)
endif()

if(DRISHTI_USE_IMSHOW)
  target_link_libraries(${test_app} imshow::imshow)
  target_compile_definitions(${test_app} PUBLIC DRISHTI_USE_IMSHOW)
endif()

target_include_directories(${test_app} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../>"
  )
set_property(TARGET ${test_app} PROPERTY FOLDER "app/console")
install(TARGETS ${test_app} DESTINATION bin)
