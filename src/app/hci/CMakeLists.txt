if(NOT DRISHTI_USE_IMSHOW)
  return() # temporary hard requirement
endif()

hunter_add_package(glfw)
find_package(glfw3 REQUIRED)

set(test_app drishti-hci)

add_executable(${test_app} hci.cpp GLWindow.h GLWindow.cpp)
target_link_libraries(${test_app} drishtisdk cxxopts::cxxopts ${OpenCV_LIBS} drishti_videoio glfw)

# imshow used for AVFoundation workaround
if(DRISHTI_USE_IMSHOW)
  target_link_libraries(${test_app} imshow::imshow)
  target_compile_definitions(${test_app} PUBLIC DRISHTI_USE_IMSHOW)
endif()

target_include_directories(${test_app} PUBLIC "$<BUILD_INTERFACE:${DRISHTI_APP_DIRECTORIES}>")

set_property(TARGET ${test_app} PROPERTY FOLDER "app/console")
install(TARGETS ${test_app} DESTINATION bin)
