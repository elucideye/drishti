include(CMakeParseArguments) # cmake_parse_arguments

# Download and unpack archive.
# Layout:
#   ${CMAKE_BINARY_DIR}/
#     _3rdParty/
#       PrivateDownloads/
#         <name>/
#           <sha1>/
#             archive.file
#             Unpacked/    # -> x_DIR
#             DONE
function(drishti_download_private_data)
  if(CMAKE_VERSION VERSION_LESS "3.7")
    message(FATAL_ERROR "CMake 3.7+ required")
  endif()
  # -> x_URL
  # -> x_SHA1
  # -> x_NAME
  # -> x_DIR
  # -> x_PASSWORDS
  cmake_parse_arguments(x "" "URL;SHA1;NAME;DIR;PASSWORDS" "" "${ARGV}")

  string(COMPARE NOTEQUAL "${x_UNPARSED_ARGUMENTS}" "" has_unparsed)
  if(has_unparsed)
    message(FATAL_ERROR "Unparsed arguments: ${x_UNPARSED_ARGUMENTS}")
  endif()

  # sanity check
  string(COMPARE EQUAL "${CMAKE_BINARY_DIR}" "" is_empty)
  if(is_empty)
    message(FATAL_ERROR "CMAKE_BINARY_DIR is empty")
  endif()

  string(COMPARE EQUAL "${x_URL}" "" is_empty)
  if(is_empty)
    message(FATAL_ERROR "URL is missing")
  endif()

  string(COMPARE EQUAL "${x_SHA1}" "" is_empty)
  if(is_empty)
    message(FATAL_ERROR "SHA1 is missing")
  endif()

  string(COMPARE EQUAL "${x_NAME}" "" is_empty)
  if(is_empty)
    message(FATAL_ERROR "NAME is missing")
  endif()

  string(COMPARE EQUAL "${x_DIR}" "" is_empty)
  if(is_empty)
    message(FATAL_ERROR "DIR is missing")
  endif()

  string(COMPARE EQUAL "${x_PASSWORDS}" "" is_empty)
  if(is_empty)
    message(FATAL_ERROR "PASSWORDS is missing")
  endif()

  if(NOT EXISTS "${x_PASSWORDS}")
    message(FATAL_ERROR "File not found: '${x_PASSWORDS}'")
  endif()

  set(data_dir "${CMAKE_BINARY_DIR}/_3rdParty/PrivateDownloads/${x_NAME}")
  set(sha1_dir "${data_dir}/${x_SHA1}")
  set(done_stamp "${sha1_dir}/DONE")
  set(unpacked_dir "${sha1_dir}/Unpacked")
  set(archive_file "${sha1_dir}/archive.file")

  if(EXISTS "${done_stamp}")
    file(GLOB res "${unpacked_dir}/*")
    set("${x_DIR}" "${res}" PARENT_SCOPE)
    return()
  endif()

  file(REMOVE_RECURSE "${data_dir}")
  file(MAKE_DIRECTORY "${unpacked_dir}")

  include("${x_PASSWORDS}")

  string(COMPARE EQUAL "${USERNAME}" "" is_empty)
  if(is_empty)
    message(FATAL_ERROR "USERNAME is not defined in '${x_PASSWORDS}' file")
  endif()

  string(COMPARE EQUAL "${PASSWORD}" "" is_empty)
  if(is_empty)
    message(FATAL_ERROR "PASSWORD is not defined in '${x_PASSWORDS}' file")
  endif()

  message("Downloading '${x_URL}'...")
  file(
      DOWNLOAD
      "${x_URL}"
      "${archive_file}"
      SHOW_PROGRESS
      STATUS status
      EXPECTED_HASH SHA1=${x_SHA1}
      USERPWD "${USERNAME}:${PASSWORD}"
  )

  list(GET status 0 error_code)
  list(GET status 1 error_message)

  if(NOT error_code EQUAL 0)
    message(FATAL_ERROR "Download failed (${error_code}; ${error_message}")
  endif()

  execute_process(
      COMMAND "${CMAKE_COMMAND}" -E tar xvf "${archive_file}"
      WORKING_DIRECTORY "${unpacked_dir}"
      RESULT_VARIABLE result
  )

  if(NOT result EQUAL 0)
    message(FATAL_ERROR "Unpacking failed")
  endif()

  file(WRITE "${done_stamp}" "")

  file(GLOB res "${unpacked_dir}/*")
  set("${x_DIR}" "${res}" PARENT_SCOPE)
endfunction()
