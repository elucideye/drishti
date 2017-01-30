function(drishti_split_debug_symbols lib_name)

  if(XCODE)

    # TODO: For iOS universal builds we need to support both: iphoneos and iphonesimulator
    # i.e., _builds/<toolchain>/src/lib/drishti/Release-iphoneos/libdrishti.dylib.dSYM
    set(dsym_name "${lib_name}.dSYM")
    add_custom_command(TARGET ${lib_name}
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_directory "$<TARGET_FILE:${dsym_name}" "${CMAKE_BINARY_DIR}/${dsym_name}"
      )

    # Install the unstripped library itself via build-id:
    install(DIRECTORY
      "${CMAKE_BINARY_DIR}/${lib_name}.dSYM"
      DESTINATION ".dSYM/"
      )

  else()

    # see: https://sourceware.org/gdb/onlinedocs/gdb/Separate-Debug-Files.html

    set(BUILDID "abcdef1234")
    string(SUBSTRING "${BUILDID}" 0 2 BUILDIDPREFIX)
    string(SUBSTRING "${BUILDID}" 2 8 BUILDIDSUFFIX)

    include(drishti_check_cxx_linker_flag)
    drishti_check_cxx_linker_flag("-Wl,--build-id=0x${BUILDID}" linker_supports_build_id)

    if(linker_supports_build_id)
      set_target_properties(${lib_name} PROPERTIES LINK_FLAGS "-Wl,--build-id=0x${BUILDID}")
      set(debug_lib "${lib_name}.debug")
      add_custom_command(TARGET ${lib_name}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:${lib_name}>" "${CMAKE_BINARY_DIR}/${debug_lib}"

        # Prefer to strip via CMake install/strip target
        COMMAND ${CMAKE_STRIP} -g $<TARGET_FILE:${lib_name}>
        )

      # Install the unstripped library itself via build-id:
      install(FILES
        "${CMAKE_BINARY_DIR}/${debug_lib}"
        DESTINATION ".build-id/${BUILDIDPREFIX}"
        RENAME "${BUILDIDSUFFIX}.debug"
        )
    endif()
  endif()

endfunction(drishti_split_debug_symbols)
