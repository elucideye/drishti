# Search all populated CMake variables matching the pattern <name>_LICENSE, and copy to specified directory
# as ${drishti_license_dir}/<name>_LICENSE

function(drishti_copy_3rdparty_licenses drishti_license_dir)
  message("drishti_license_dir : ${drishti_license_dir}")
  get_cmake_property(_variableNames VARIABLES)
  foreach (_variableName ${_variableNames})
    STRING(REGEX MATCH "^.*_LICENSES$" _licenseFiles "${_variableName}")
    if(NOT ${_licenseFiles} STREQUAL "")
      message("VARIABLE: ${_variableName}=${${_variableName}}")
      if(EXISTS "${${_variableName}}")
        install(FILES ${${_variableName}} DESTINATION ${drishti_license_dir} RENAME ${_licenseFiles})
      else()
        message(WARNING "Missing license file : ${_variableName}=${${_variableName}}")
      endif()
    endif()
  endforeach()
endfunction()
