include(CMakeParseArguments) # cmake_parse_arguments

include(drishti_get_library_location)

function(drishti_merge_libraries_msvc)
  set(optional)
  set(one FINAL)
  set(multiple ALL_DEPENDENCIES)

  # Introduce:
  # * x_FINAL
  # * x_ALL_DEPENDENCIES
  cmake_parse_arguments(x "${optional}" "${one}" "${multiple}" "${ARGV}")

  string(COMPARE NOTEQUAL "${x_UNPARSED_ARGUMENTS}" "" has_unparsed)
  if(has_unparsed)
    message(FATAL_ERROR "Unparsed arguments: ${x_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT TARGET "${x_FINAL}")
    message(FATAL_ERROR "Target doesn't exist: ${x_FINAL}")
  endif()

  string(COMPARE EQUAL "${CMAKE_CFG_INTDIR}" "." is_single)
  if(is_single)
    set(extra_linker_flags "")
    foreach(x ${x_ALL_DEPENDENCIES})
      drishti_get_library_location(
          LIBRARY "${x}"
          CONFIG "${CMAKE_BUILD_TYPE}"
          RESULT location
      )
      set(extra_linker_flags "${extra_linker_flags} ${location}")
    endforeach()

    set_target_properties(
        "${x_FINAL}" PROPERTIES STATIC_LIBRARY_FLAGS "${extra_linker_flags}"
    )
    return()
  endif()

  foreach(config ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER "${config}" config_upper)

    set(extra_linker_flags "")
    foreach(x ${x_ALL_DEPENDENCIES})
      drishti_get_library_location(
          LIBRARY "${x}"
          CONFIG "${config_upper}"
          RESULT location
      )
      set(extra_linker_flags "${extra_linker_flags} ${location}")
    endforeach()

    set_target_properties(
        "${x_FINAL}" PROPERTIES STATIC_LIBRARY_FLAGS_${config_upper} "${extra_linker_flags}"
    )
  endforeach()
endfunction()
