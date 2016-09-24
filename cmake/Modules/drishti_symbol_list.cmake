function(drishti_symbol_list drishti_library)
  set(
    DRISHTI_SDK_SYMBOLS
    _drishti_create_from_file
    _drishti_create_from_stream
    _drishti_destroy
    _drishti_segment
    )

  set (LINK_FLAGS "")

  if (APPLE)
    # Create a symbols_list file for the darwin linker
    string(REPLACE ";" "\n_" _symbols "${SDK_SYMBOLS}")
    set(_symbols_list "${CMAKE_CURRENT_BINARY_DIR}/symbols.list")
    file(WRITE ${_symbols_list} "_${_symbols}\n")
    set(LINK_FLAGS "${LINK_FLAGS} -Wl,-exported_symbols_list,'${_symbols_list}'")
  elseif (CMAKE_C_COMPILER_ID STREQUAL GNU)
    # Create a version script for GNU ld.
    set(_symbols "{ global: ${SDK_SYMBOLS}; local: *; };")
    set(_version_script "${CMAKE_CURRENT_BINARY_DIR}/version.script")
    file(WRITE ${_version_script} "${_symbols}\n")
    set(LINK_FLAGS "${LINK_FLAGS} -Wl,--version-script,'${_version_script}'")
  endif (APPLE)

  set_target_properties(
    ${drishti_library}
    PROPERTIES
    LINK_FLAGS "${LINK_FLAGS}"
    )

endfunction(drishti_symbol_list)
