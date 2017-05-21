hunter_add_package(Boost)
find_package(Boost CONFIG REQUIRED)

set(drishti_gltest_libs Boost::boost)

set(drishti_gltest_srcs
  drishti_gltest.h
  GLContext.h
  GLContext.cpp)
  
if(DRISHTI_BUILD_QT)
  find_package(Qt5PrintSupport REQUIRED)
  list(APPEND drishti_gltest_libs Qt5::Widgets Qt5::PrintSupport)
  list(APPEND drishti_gltest_srcs QGLContext.cpp QGLContext.h)
endif()

message("drishti_is_mobile:${drishti_is_mobile}")
if(NOT ${drishti_is_mobile})
  # Include glfw for lightweight hidden window opengl context:
  # Alternatives: Qt or glfm for iOS and Android
  set(drishti_has_glfw TRUE)   
  hunter_add_package(glfw)
  find_package(glfw3 REQUIRED)
  list(APPEND drishti_gltest_libs glfw)
  list(APPEND drishti_gltest_srcs GLFWContext.cpp GLFWContext.h)
  message("drishti_has_glfw: ${drishti_has_glfw}")
endif()

add_library(drishti_gltest ${drishti_gltest_srcs})
target_link_libraries(drishti_gltest PUBLIC ${drishti_gltest_libs})

if(DRISHTI_BUILD_QT)
  target_compile_definitions(drishti_gltest PUBLIC DRISHTI_BUILD_QT=1)
endif()

if(${drishti_has_glfw})
  target_compile_definitions(drishti_gltest PUBLIC DRISHTI_HAS_GLFW=1)
endif()

target_include_directories(drishti_gltest PUBLIC "$<BUILD_INTERFACE:${DRISHTI_INCLUDE_DIRECTORIES}>")
set_property(TARGET drishti_gltest PROPERTY FOLDER "libs/drishti")
