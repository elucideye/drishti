function(drishti_configure_console_app
    DRISHTI_SDK_PROJECT_NAME
    DRISHTI_SDK_PRODUCT_NAME
    DRISHTI_SDK_APP_NAME
    DRISHTI_SDK_SOURCES
    DRISHTI_SDK_LIBS)

  message("DRISHTI_SDK_LIBS: ${DRISHTI_SDK_LIBS}")

  string(COMPARE EQUAL "${CMAKE_OSX_SYSROOT}" "iphoneos" is_ios)

  if(is_ios)
    # Simple iOS console application
    configure_file("${CMAKE_SOURCE_DIR}/src/app/ios/boilerplate/empty_application/CMakeLists.txt.in" ${DRISHTI_SDK_PROJECT_NAME}.cmake @ONLY)
    include("${CMAKE_CURRENT_BINARY_DIR}/${DRISHTI_SDK_PROJECT_NAME}.cmake")
  else() ### ANDROID or *
    # Simple *nix console application
    add_executable(${DRISHTI_SDK_APP_NAME} ${DRISHTI_SDK_SOURCES})
    target_link_libraries (${DRISHTI_SDK_APP_NAME} ${DRISHTI_SDK_LIBS})
    install(TARGETS ${DRISHTI_SDK_APP_NAME} DESTINATION bin)
    set_target_properties(${DRISHTI_SDK_APP_NAME} PROPERTIES INSTALL_RPATH "@loader_path/../lib")
  endif()

endfunction(drishti_configure_console_app)
