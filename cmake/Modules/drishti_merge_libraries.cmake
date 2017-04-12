include(CMakeParseArguments) # cmake_parse_arguments

cmake_policy(SET CMP0026 OLD) # drishti_merge_libraries_msvc use LOCATION

include(drishti_merge_get_all_dependencies)
include(drishti_merge_libraries_ar_ranlib)
include(drishti_merge_libraries_msvc)
include(drishti_merge_libraries_xcode)

set(DRISHTI_MERGE_LIBRARIES_DIR "${CMAKE_CURRENT_LIST_DIR}")

function(drishti_merge_libraries)
  set(optional)
  set(one FINAL)
  set(multiple LIBRARIES)

  # Introduce:
  # * x_FINAL
  # * x_LIBRARIES
  cmake_parse_arguments(x "${optional}" "${one}" "${multiple}" "${ARGV}")

  string(COMPARE NOTEQUAL "${x_UNPARSED_ARGUMENTS}" "" has_unparsed)
  if(has_unparsed)
    message(FATAL_ERROR "Unparsed arguments: ${x_UNPARSED_ARGUMENTS}")
  endif()

  if(TARGET "${x_FINAL}")
    message(FATAL_ERROR "Target already exists: ${x_FINAL}")
  endif()

  set(working_dir "${CMAKE_CURRENT_BINARY_DIR}/_MergeLibraries")
  set(libraries_list "${working_dir}/libraries.list")

  add_library("${x_FINAL}" ${DRISHTI_MERGE_LIBRARIES_DIR}/source/foo.cpp)

  drishti_merge_get_all_dependencies(
      INPUT ${x_LIBRARIES}
      OUTPUT all_dependencies
  )

  if(NOT all_dependencies)
    message(FATAL_ERROR "The list of dependencies is empty")
  endif()

  foreach(x ${all_dependencies})
    if(TARGET "${x}")
      add_dependencies("${x_FINAL}" "${x}")
    endif()
  endforeach()

  drishti_merge_print("Creating library '${x_FINAL}' from libraries:")
  foreach(x ${all_dependencies})
    drishti_merge_print(" * ${x}")
  endforeach()

  if(MSVC)
    drishti_merge_libraries_msvc(
        ALL_DEPENDENCIES
        ${all_dependencies}
        FINAL
        ${x_FINAL}
    )
  elseif(XCODE)
    drishti_merge_libraries_xcode(
        ALL_DEPENDENCIES
        ${all_dependencies}
        LIBRARIES_LIST
        ${libraries_list}
        FINAL
        ${x_FINAL}
        WORKING_DIR
        ${working_dir}
    )
  else()
    drishti_merge_libraries_ar_ranlib(
        ALL_DEPENDENCIES
        ${all_dependencies}
        LIBRARIES_LIST
        ${libraries_list}
        FINAL
        ${x_FINAL}
        WORKING_DIR
        ${working_dir}
    )
  endif()
endfunction()
