#https://cmake.org/Wiki/CMake_RPATH_handling

macro(drishti_set_rpath)
  set(DRISHTI_ORIGIN "$ORIGIN")
  if (APPLE)
    set(DRISHTI_ORIGIN "@loader_path")
  endif()
  set(CMAKE_INSTALL_RPATH "${DRISHTI_ORIGIN}/../lib:${DRISHTI_ORIGIN}/")
endmacro()
