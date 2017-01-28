# CMake style would be: CheckCxxLinkerFlag, but this could very well be added in the future
# Note that CheckCxxCompilerFlags will not work for linker flags.
# See discussion here: https://cmake.org/pipermail/cmake/2011-July/045525.html
function(drishti_check_cxx_linker_flag _linker_flags _resultVar)
  set(CMAKE_REQUIRED_FLAGS_ "${CMAKE_REQUIRED_FLAGS}") # stash flags

  set(CMAKE_REQUIRED_FLAGS "${_linker_flags}")
  message(STATUS "Check linker flag -- test linker flags: ${CMAKE_REQUIRED_FLAGS}")

  include(CheckCXXSourceCompiles)
  check_cxx_source_compiles("int main() { return 0; }" _result)
  set(${_resultVar} ${_result} PARENT_SCOPE)
  
  set(CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS_}") # restore flags
endfunction()
