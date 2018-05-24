#https://cmake.org/Wiki/CMake_RPATH_handling

macro(drishti_set_rpath)
  if (APPLE)
    set(DRISHTI_ORIGIN "@loader_path")
  else()
    set(DRISHTI_ORIGIN "$ORIGIN")
  endif()
  set(CMAKE_INSTALL_RPATH "${DRISHTI_ORIGIN}/../lib" "${DRISHTI_ORIGIN}")
endmacro()
