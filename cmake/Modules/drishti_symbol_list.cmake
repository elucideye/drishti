function(drishti_symbol_list drishti_library)
  set(
    DRISHTI_SDK_SYMBOLS
    drishti_eye_segmenter_create_from_file
    drishti_eye_segmenter_create_from_stream
    drishti_eye_segmenter_destroy
    drishti_eye_segmenter_segment
    )

  if(DRISHTI_BUILD_HCI)
    list(APPEND DRISHTI_SDK_SYMBOLS
      drishti_face_tracker_create_from_streams
      drishti_face_tracker_destroy
      drishti_face_tracker_track
      drishti_face_tracker_callback
      )
  endif()

  get_target_property(x ${drishti_library} LINK_FLAGS)
  if(x)
    message(FATAL_ERROR "No LINK_FLAGS expected")
  endif()  

  if (APPLE)
    # Create a symbols_list file for the darwin linker
    string(REPLACE ";" "\n_" _symbols "${DRISHTI_SDK_SYMBOLS}")
    set(_symbols_list "${CMAKE_CURRENT_BINARY_DIR}/symbols.list")
    file(WRITE ${_symbols_list} "_${_symbols}\n")
    set(LINK_FLAGS "${LINK_FLAGS} -Wl,-exported_symbols_list,'${_symbols_list}'")    

    # Support xcode toolchain use XCODE_ATTRIBUTE_*
    set_target_properties(${drishti_library}
      PROPERTIES
      XCODE_ATTRIBUTE_EXPORTED_SYMBOLS_FILE "${_symbols_list}"
      XCODE_ATTRIBUTE_COPY_PHASE_STRIP "YES"
      XCODE_ATTRIBUTE_STRIP_INSTALLED_PRODUCT "YES"
      XCODE_ATTRIBUTE_STRIP_STYLE "non-global"
      XCODE_ATTRIBUTE_STRIPFLAGS "-x -u -r"
      XCODE_ATTRIBUTE_DEAD_CODE_STRIPPING "YES"
      XCODE_ATTRIBUTE_DEPLOYMENT_POSTPROCESSING "YES"
      XCODE_ATTRIBUTE_GENERATE_MASTER_OBJECT_FILE "YES" # "Perform Single-Object Prelink"
      )

  elseif (CMAKE_C_COMPILER_ID STREQUAL GNU)
    # Create a version script for GNU ld.
    set(_symbols "{ global: ${DRISHTI_SDK_SYMBOLS}; local: *; };")
    set(_version_script "${CMAKE_CURRENT_BINARY_DIR}/version.script")
    file(WRITE ${_version_script} "${_symbols}\n")
    set(LINK_FLAGS "${LINK_FLAGS} -Wl,--version-script,'${_version_script}'")
  endif (APPLE)

  # TODO: MSVC, etc

  set_target_properties(
    ${drishti_library}
    PROPERTIES
    LINK_FLAGS "${LINK_FLAGS}"
    )

endfunction(drishti_symbol_list)
