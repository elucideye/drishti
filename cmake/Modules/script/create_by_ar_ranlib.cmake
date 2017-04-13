cmake_minimum_required(VERSION 3.1)

include("${CMAKE_CURRENT_LIST_DIR}/../drishti_merge_print.cmake")

if("${LIBRARIES_LIST}" STREQUAL "")
  message(FATAL_ERROR "LIBRARIES_LIST is empty")
endif()

if(NOT EXISTS "${LIBRARIES_LIST}")
  message(FATAL_ERROR "File not found: '${LIBRARIES_LIST}'")
endif()

if("${WORKING_DIR}" STREQUAL "")
  message(FATAL_ERROR "WORKING_DIR is empty")
endif()

if(NOT EXISTS "${WORKING_DIR}")
  message(FATAL_ERROR "Directory not found: '${WORKING_DIR}'")
endif()

if("${CMAKE_AR}" STREQUAL "")
  message(FATAL_ERROR "CMAKE_AR is empty")
endif()

if(NOT EXISTS "${CMAKE_AR}")
  message(FATAL_ERROR "'ar' not found: '${CMAKE_AR}'")
endif()

if("${CMAKE_RANLIB}" STREQUAL "")
  message(FATAL_ERROR "CMAKE_RANLIB is empty")
endif()

if(NOT EXISTS "${CMAKE_RANLIB}")
  message(FATAL_ERROR "'ranlib' not found: '${CMAKE_RANLIB}'")
endif()

if("${FINAL_LIBRARY}" STREQUAL "")
  message(FATAL_ERROR "FINAL_LIBRARY is empty")
endif()

if(NOT EXISTS "${FINAL_LIBRARY}")
  message(FATAL_ERROR "Library not found: '${FINAL_LIBRARY}'")
endif()

file(STRINGS "${LIBRARIES_LIST}" libraries_list)

set(disintegration_top "${WORKING_DIR}/disintegration")

file(REMOVE "${FINAL_LIBRARY}")

foreach(x ${libraries_list})
  if(NOT EXISTS "${x}")
    message(FATAL_ERROR "Library not found: '${x}'")
  endif()
  get_filename_component(lib_name "${x}" NAME)

  set(disintegration_dir "${disintegration_top}/${lib_name}")
  if(EXISTS "${disintegration_dir}")
    message(
        FATAL_ERROR
        "Directory already exists: '${disintegration_dir}' (library name duplicate)"
    )
  endif()
  file(MAKE_DIRECTORY "${disintegration_dir}")

  # Verify that there are no two objects with the same name in archive
  # * http://www.linux.org/threads/same-filename-o-in-a.8834/
  # * http://stackoverflow.com/a/3821949/2288008 (second comment)
  set(cmd "${CMAKE_AR}" t "${x}")
  execute_process(
      COMMAND
      ${cmd}
      RESULT_VARIABLE result
      OUTPUT_VARIABLE ar_t_output
      OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "Command failed: " ${cmd})
  endif()
  string(REPLACE "\n" " " list_of_objects "${ar_t_output}")
  separate_arguments(list_of_objects)
  list(LENGTH list_of_objects original_len)
  list(REMOVE_DUPLICATES list_of_objects)
  list(LENGTH list_of_objects modified_len)
  if(NOT original_len EQUAL modified_len)
    message(
        FATAL_ERROR
        "Unsupported: objects with the same name detected"
        " in library '${x}': ${ar_t_output}"
    )
  endif()

  set(cmd "${CMAKE_AR}" x "${x}")
  execute_process(
      COMMAND
      ${cmd}
      WORKING_DIRECTORY "${disintegration_dir}"
      RESULT_VARIABLE result
  )
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "Command failed " ${cmd})
  endif()

  if(NOT EXISTS "${FINAL_LIBRARY}")
    drishti_merge_print("Creating library:")
    drishti_merge_print("* '${FINAL_LIBRARY}'")
    drishti_merge_print("From objects of:")
    drishti_merge_print("* '${x}'")
    set(flags "qc")
  else()
    drishti_merge_print("Appending objects of:")
    drishti_merge_print("* '${x}'")
    set(flags "q")
  endif()

  file(
      GLOB
      list_of_objects
      LIST_DIRECTORIES false
      "${disintegration_dir}/*"
  )

  set(cmd ${CMAKE_AR} ${flags} ${FINAL_LIBRARY} ${list_of_objects})
  execute_process(
      COMMAND
      ${cmd}
      WORKING_DIRECTORY "${disintegration_dir}"
      RESULT_VARIABLE result
  )
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "Command failed " ${cmd})
  endif()
endforeach()

set(cmd ${CMAKE_RANLIB} ${FINAL_LIBRARY})
execute_process(
    COMMAND
    ${cmd}
    WORKING_DIRECTORY "${WORKING_DIR}"
    RESULT_VARIABLE result
)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "Command failed " ${cmd})
endif()

drishti_merge_print("Done:")
drishti_merge_print("* '${FINAL_LIBRARY}'")
