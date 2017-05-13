set(test_app drishti-face)

add_executable(${test_app} face.cpp)
target_link_libraries(${test_app} drishtisdk cxxopts::cxxopts ${OpenCV_LIBS} drishti_videoio)

if(DRISHTI_USE_IMSHOW)
  target_link_libraries(${test_app} imshow::imshow)
  target_compile_definitions(${test_app} PUBLIC DRISHTI_USE_IMSHOW)
endif()

#include "videoio/VideoSource.h"
target_include_directories(${test_app} PUBLIC "$<BUILD_INTERFACE:${DRISHTI_APP_DIRECTORIES}>")

set_property(TARGET ${test_app} PROPERTY FOLDER "app/console")
install(TARGETS ${test_app} DESTINATION bin)
