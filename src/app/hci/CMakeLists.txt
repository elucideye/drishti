set(test_app drishti-hci)

hunter_add_package(gauze)
find_package(gauze CONFIG REQUIRED)

add_executable(${test_app} hci.cpp)
target_link_libraries(${test_app} PUBLIC
  drishtisdk
  cxxopts::cxxopts
  ${OpenCV_LIBS}
  drishti_videoio
  aglet::aglet
  gauze::gauze)

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
endif()

target_include_directories(${test_app} PUBLIC "$<BUILD_INTERFACE:${DRISHTI_APP_DIRECTORIES}>")
set_property(TARGET ${test_app} PROPERTY FOLDER "app/console")
install(TARGETS ${test_app} DESTINATION bin)
