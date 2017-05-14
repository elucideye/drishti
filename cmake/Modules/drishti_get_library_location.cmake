include(CMakeParseArguments) # cmake_parse_arguments

function(drishti_get_library_location)
  set(optional)
  set(one LIBRARY CONFIG RESULT)

  # Introduce:
  # * x_LIBRARY
  # * x_CONFIG
  # * x_RESULT
  cmake_parse_arguments(x "${optional}" "${one}" "${multiple}" "${ARGV}")

  string(COMPARE NOTEQUAL "${x_UNPARSED_ARGUMENTS}" "" has_unparsed)
  if(has_unparsed)
    message(FATAL_ERROR "Unparsed arguments: ${x_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT TARGET "${x_LIBRARY}")
    set("${x_RESULT}" "${x_LIBRARY}" PARENT_SCOPE)
    return()
  endif()

  string(TOUPPER "${x_CONFIG}" config_upper)

  get_target_property(location "${x_LIBRARY}" LOCATION_${config_upper})
  if(location)
    set("${x_RESULT}" "${location}" PARENT_SCOPE)
    return()
  endif()

  get_target_property(location "${x_LIBRARY}" LOCATION)
  if(NOT location)
    message(FATAL_ERROR "Cannot determine location of librari '${x_LIBRARY}'")
  endif()

  set("${x_RESULT}" "${location}" PARENT_SCOPE)
endfunction()
