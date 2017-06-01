set(test_app drishti-hci)

add_executable(${test_app} hci.cpp)
target_link_libraries(${test_app} PUBLIC
  drishtisdk
  cxxopts::cxxopts
  ${OpenCV_LIBS}
  drishti_videoio
  drishti_gltest
  drishti_test_main)

# Get toolchain suffix:
if(EXISTS "${CMAKE_TOOLCHAIN_FILE}")
  get_filename_component(toolchain_suffix "${CMAKE_TOOLCHAIN_FILE}" NAME_WE)
else()
  set(toolchain_suffix "default")
endif()

if(DRISHTI_IS_MOBILE)
  target_compile_definitions(${test_app} PUBLIC DRISHTI_IS_MOBILE=1)
  if(IOS)
    set(BUNDLE_ID "com.elucideye.drishti.${toolchain_suffix}.${test_app}")
    message("BUNDLE_ID = ${BUNDLE_ID}")
    set_target_properties(
      "${test_app}"
      PROPERTIES
      MACOSX_BUNDLE_GUI_IDENTIFIER "${BUNDLE_ID}"
      )
  endif()
else() # !DRISHTI_IS_MOBILE
  hunter_add_package(glfw)
  find_package(glfw3 REQUIRED)

  add_library(drishti_glview GLWindow.h GLWindow.cpp)
  target_link_libraries(drishti_glview PUBLIC glfw ${OpenCV_LIBS})
  target_link_libraries(${test_app} PUBLIC drishti_glview)
  set_property(TARGET ${test_app} PROPERTY FOLDER "libs/drishti")
endif()

target_include_directories(${test_app} PUBLIC "$<BUILD_INTERFACE:${DRISHTI_APP_DIRECTORIES}>")
set_property(TARGET ${test_app} PROPERTY FOLDER "app/console")
install(TARGETS ${test_app} DESTINATION bin)
