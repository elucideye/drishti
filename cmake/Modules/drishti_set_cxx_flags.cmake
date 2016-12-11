macro(drishti_set_cxx_flags)

  if(NOT MSVC)

    # -Wno-narrowing  = Don't warn about type narrowing
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG(-Wno-c++11-narrowing COMPILER_SUPPORTS_NO_NARROWING)
    if(COMPILER_SUPPORTS_NO_NARROWING)
      set(CMAKE_CXX_FLAGS    "${CMAKE_CXX_FLAGS} -Wno-c++11-narrowing")
    endif()

    CHECK_CXX_COMPILER_FLAG(-Wno-unused-parameter COMPILER_SUPPORTS_NO_UNUSED_PARAMETER)
    if(COMPILER_SUPPORTS_NO_UNUSED_PARAMETER)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")
    endif()

    CHECK_CXX_COMPILER_FLAG(-Wno-unused-local-typedefs COMPILER_SUPPORTS_NO_UNUSED_LOCAL_TYPEDEFS)
    if(COMPILER_SUPPORTS_NO_UNUSED_LOCAL_TYPEDEFS)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-local-typedefs")
    endif()

    CHECK_CXX_COMPILER_FLAG(-Wno-unknown-attributes COMPILER_SUPPORTS_NO_UNKNOWN_ATTRIBUTES)
    if(COMPILER_SUPPORTS_NO_UNKNOWN_ATTRIBUTES)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unknown-attributes")
    endif()

    CHECK_CXX_COMPILER_FLAG(-Wno-shorten-64-to-32 COMPILER_SUPPORTS_NO_SHORTEN_64_TO_32)
    if(COMPILER_SUPPORTS_NO_SHORTEN_64_TO_32)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-shorten-64-to-32")
    endif()

  endif()

endmacro()
