hunter_add_package(Boost)
find_package(Boost CONFIG REQUIRED)

set(drishti_gltest_libs Boost::boost)
set(drishti_gltest_defs "")
set(drishti_gltest_srcs
  drishti_gltest.h
  GLContext.h
  GLContext.cpp)

if(ANDROID)
  find_library(log-lib log)
  find_library(android-lib android)
  find_library(EGL-lib EGL)
  find_library(GLESv2-lib GLESv2)
  list(APPEND drishti_gltest_srcs GLContextAndroid.cpp GLContextAndroid.h)
  list(APPEND drishti_gltest_defs DRISHTI_ANDROID=1)  
  list(APPEND drishti_gltest_libs ${log-lib} ${android-lib} ${EGL-lib} ${GLESv2-lib})
endif()

if(IOS)
  list(APPEND drishti_gltest_srcs GLContextIOS.mm GLContextIOS.h)
  list(APPEND drishti_gltest_defs DRISHTI_IOS=1)  
  list(APPEND drishti_gltest_libs "-framework OpenGLES")
endif()

if(NOT ${DRISHTI_IS_MOBILE})
  hunter_add_package(glfw)
  find_package(glfw3 REQUIRED)
  list(APPEND drishti_gltest_srcs GLFWContext.cpp GLFWContext.h)
  list(APPEND drishti_gltest_defs DRISHTI_HAS_GLFW=1)    
  list(APPEND drishti_gltest_libs glfw)
endif()

add_library(drishti_gltest ${drishti_gltest_srcs})
target_link_libraries(drishti_gltest PUBLIC ${drishti_gltest_libs})
target_compile_definitions(drishti_gltest PUBLIC ${drishti_gltest_defs})
target_include_directories(drishti_gltest PUBLIC "$<BUILD_INTERFACE:${DRISHTI_INCLUDE_DIRECTORIES}>")
set_property(TARGET drishti_gltest PROPERTY FOLDER "libs/drishti")
