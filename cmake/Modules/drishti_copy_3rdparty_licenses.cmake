# Search all populated CMake variables matching the pattern <name>_LICENSE, and copy to specified directory
# as ${drishti_license_dir}/<name>_LICENSE

function(drishti_copy_3rdparty_licenses drishti_license_dir)
  message("drishti_license_dir : ${drishti_license_dir}")
  get_cmake_property(_variableNames VARIABLES)
  add_custom_command(TARGET drishtisdk POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${drishti_license_dir}"
    COMMENT "Create directory ${drishti_license_dir}"
    VERBATIM
    )
  foreach (_variableName ${_variableNames})
    STRING(REGEX MATCH "^.*_LICENSES$" _licenseFiles "${_variableName}")
    if(NOT ${_licenseFiles} STREQUAL "")
      message("VARIABLE: ${_variableName}=${${_variableName}}")
      if(EXISTS "${${_variableName}}")
        add_custom_command(TARGET drishtisdk POST_BUILD
          COMMAND ${CMAKE_COMMAND} -E copy_if_different "${${_variableName}}" "${drishti_license_dir}/${_licenseFiles}"
          DEPENDS "${${_variableName}}"
          COMMENT "Copying ${${_variableName}} to ${drishti_license_dir}/${_licenseFiles}"
          VERBATIM
          )
      else()
        message(WARNING "Missing license file : ${_variableName}=${${_variableName}}")
      endif()
    endif()
  endforeach()
endfunction()
