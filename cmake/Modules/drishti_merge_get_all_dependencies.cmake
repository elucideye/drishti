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
    get_target_property(_target_type "${x}" TYPE)
    if(_target_type STREQUAL "INTERFACE_LIBRARY")
      continue()
    endif()
    get_property(link_libraries TARGET ${x} PROPERTY LINK_LIBRARIES)
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
    set("${x_OUTPUT}" ${new_already_processed} PARENT_SCOPE)
    return()
  endif()

  drishti_merge_get_all_dependencies(
      INPUT ${new_libraries}
      OUTPUT output
      ALREADY_PROCESSED ${new_already_processed}
  )

  set("${x_OUTPUT}" ${output} PARENT_SCOPE)
endfunction()
