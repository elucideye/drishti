macro(PARALLEL_MAKE)

# Macro from here: http://stackoverflow.com/questions/10688549/cmake-and-parallel-build
# Enable via CL i.e., -DMULTITHREADED_BUILD=8

#Add some multithreaded build support
MARK_AS_ADVANCED(MULTITHREADED_BUILD)
if(MULTITHREADED_BUILD)
    if(${CMAKE_GENERATOR} MATCHES "Unix Makefiles")
            message(STATUS ${CMAKE_BUILD_TOOL})
            set(CMAKE_MAKE_PROGRAM "${CMAKE_MAKE_PROGRAM} -j${MULTITHREADED_BUILD}")
            message(STATUS "Added arguments to CMAKE_BUILD_TOOL: ${CMAKE_MAKE_PROGRAM}")
    elseif(MSVC)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
      message(STATUS "Added parallel build arguments to CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
    endif()
endif()

endmacro(PARALLEL_MAKE)