include(drishti_merge_print)

include(CMakeParseArguments) # cmake_parse_arguments

function(drishti_merge_get_all_dependencies)
  set(optional)
  set(one OUTPUT)
  set(multiple INPUT ALREADY_PROCESSED)

  # Introduce:
  # * x_OUTPUT
  # * x_INPUT
  # * x_ALREADY_PROCESSED
  cmake_parse_arguments(x "${optional}" "${one}" "${multiple}" "${ARGV}")

  string(COMPARE NOTEQUAL "${x_UNPARSED_ARGUMENTS}" "" has_unparsed)
  if(has_unparsed)
    message(FATAL_ERROR "Unparsed arguments: ${x_UNPARSED_ARGUMENTS}")
  endif()

  drishti_merge_print("Searching dependencies for libraries:")
  foreach(x ${x_INPUT})
    drishti_merge_print(" * ${x}")
  endforeach()

  set(new_libraries)

  foreach(x ${x_INPUT})
    if(${x} IN_LIST x_ALREADY_PROCESSED)
      continue()
    endif()
    if(NOT TARGET ${x})
      continue()
    endif()

    set(link_libraries_property LINK_LIBRARIES)
    get_target_property(target_type "${x}" TYPE)
    if(target_type STREQUAL "INTERFACE_LIBRARY")
      set(link_libraries_property INTERFACE_LINK_LIBRARIES)
    endif()

    get_property(link_libraries TARGET ${x} PROPERTY ${link_libraries_property})
    if(NOT link_libraries)
      continue()
    endif()
    list(APPEND new_libraries ${link_libraries})
    drishti_merge_print("Dependencies for library '${x}':")
    foreach(y ${link_libraries})
      drishti_merge_print(" * ${y}")
    endforeach()
  endforeach()

  set(
      new_already_processed
      ${x_ALREADY_PROCESSED}
      ${x_INPUT}
  )

  list(REMOVE_DUPLICATES new_already_processed)
  list(SORT new_already_processed)

  if(NOT new_libraries)
    # Exclude INTERFACE_LIBRARY targets
    set(result_list)
    foreach(x ${new_already_processed})
      if(TARGET ${x})
        get_target_property(target_type "${x}" TYPE)
        if(target_type STREQUAL "STATIC_LIBRARY")
          list(APPEND result_list ${x})
        endif()
      else()
        if(x MATCHES "^/.*/${CMAKE_SHARED_LIBRARY_PREFIX}[-_a-zA-Z0-9]+${CMAKE_SHARED_LIBRARY_SUFFIX}$")
          # skip shared libraries
        elseif(x MATCHES "^-l[a-zA-Z0-9]+$")
          # skip standard libraries added by '-l' flags
        elseif(x MATCHES "^-framework [a-zA-Z0-9]+$")
          # skip frameworks
        else()
          list(APPEND result_list ${x})
        endif()
      endif()
    endforeach()

    set("${x_OUTPUT}" ${result_list} PARENT_SCOPE)
    return()
  endif()

  drishti_merge_get_all_dependencies(
      INPUT ${new_libraries}
      OUTPUT output
      ALREADY_PROCESSED ${new_already_processed}
  )

  set("${x_OUTPUT}" ${output} PARENT_SCOPE)
endfunction()
