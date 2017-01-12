cmake_minimum_required(VERSION 3.3) # IN_LIST

function(drishti_get_all_dependencies)
  set(optional "")
  set(one TARGET RESULT)
  set(multiple IGNORE)

  # * x_TARGET
  # * x_RESULT
  # * x_IGNORE
  cmake_parse_arguments(x "${optional}" "${one}" "${multiple}" "${ARGV}")

  string(COMPARE NOTEQUAL "${x_UNPARSED_ARGUMENTS}" "" has_unparsed)
  if(has_unparsed)
    message(FATAL_ERROR "Unparsed arguments: ${x_UNPARSED_ARGUMENTS}")
  endif()

  if("${x_TARGET}" STREQUAL "")
    message(FATAL_ERROR "Expected TARGET")
  endif()

  if("${x_RESULT}" STREQUAL "")
    message(FATAL_ERROR "Expected RESULT")
  endif()

  if(NOT TARGET "${x_TARGET}")
    message(FATAL_ERROR "Not a target: '${x_TARGET}'")
  endif()

  set(ignoring "${x_TARGET};${x_IGNORE}")

  get_target_property(linked_libs "${x_TARGET}" INTERFACE_LINK_LIBRARIES)

  if(NOT linked_libs)
    set("${x_RESULT}" "" PARENT_SCOPE)
    return()
  endif()

  set(analyze_libs "")
  foreach(x ${linked_libs})
    if(TARGET "${x}")
      if(NOT "${x}" IN_LIST ignoring)
        list(APPEND analyze_libs "${x}")
      endif()
    endif()
  endforeach()

  set(final_deps "${analyze_libs}")

  foreach(x ${analyze_libs})
    drishti_get_all_dependencies(TARGET "${x}" IGNORE ${ignoring} RESULT deps)
    list(APPEND final_deps ${deps})
  endforeach()

  list(REMOVE_DUPLICATES final_deps)

  set("${x_RESULT}" "${final_deps}" PARENT_SCOPE)
endfunction()
