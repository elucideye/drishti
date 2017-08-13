set(test_app drishti-acf)

add_executable(${test_app} acf.cpp)
target_link_libraries(${test_app} drishtisdk cxxopts::cxxopts ${OpenCV_LIBS} drishti_videoio)

if(DRISHTI_USE_IMSHOW)
  target_link_libraries(${test_app} imshow::imshow)
  target_compile_definitions(${test_app} PUBLIC DRISHTI_USE_IMSHOW)
endif()

#include "videoio/VideoSource.h"
target_include_directories(${test_app} PUBLIC "$<BUILD_INTERFACE:${DRISHTI_APP_DIRECTORIES}>")

set_property(TARGET ${test_app} PROPERTY FOLDER "app/console")
install(TARGETS ${test_app} DESTINATION bin)


###############
### mat2cpb ###
###############

set(conv_app drishti-acf-mat2cpb)

if(DRISHTI_SERIALIZE_WITH_CVMATIO)
  add_executable(${conv_app} mat2cpb.cpp)
  target_link_libraries(${conv_app} drishtisdk cxxopts::cxxopts)
  set_property(TARGET ${conv_app} PROPERTY FOLDER "app/console")
  install(TARGETS ${conv_app} DESTINATION bin)
endif()
