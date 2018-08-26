################################################################################
# This directory tree contains examples built on the dependency free public SDK
# contained in src/lib/drishti/drishti.  This layer contains various simple
# primitive types to support easy integration as a shared library
# (dynamic framework) without requiring OpenCV types or types from other
# dependencies which may change.  No dynamic memory allocations are exported
# across this API line (including STL to avoid libc++ vs libstdc++ conflicts,
# etc).
################################################################################

cmake_minimum_required(VERSION 3.9) # CMAKE_INTERPROCEDURAL_OPTIMIZATION

if(NOT TARGET drishti::drishti)

  include("cmake/include/init.cmake")

  project(drishti_hunter_test VERSION 0.0.1)

  include("cmake/include/init-languages.cmake")  # must come after project()

  # * If build as a child project then parent will set this to 'OFF' (see
  #   config.cmake which will be loaded on first 'hunter_add_package')
  # * If build as a stand alone project then option will be 'ON' and 'GIT_SELF'
  #   will be used for Drishti
  option(DRISHTI_DRISHTI_AS_SELF "Use drishti as GIT_SELF" ON)

  # add the drishti package, possibly using GIT_SELF feature

  if(DEFINED drishti_FRAMEWORK)

    set(x_fmwk_path ${drishti_FRAMEWORK}) # /some/path/to/foo.framewori/foo 
    get_filename_component(x_fmwk_lib ${x_fmwk_path} NAME_WE)   # foo
    get_filename_component(x_fmwk_dir ${x_fmwk_path} DIRECTORY) # /some/path/to/foo.framework
    get_filename_component(x_fmwk_name ${x_fmwk_dir} NAME)      # foo.framework
    
    # import library
    add_library(${x_fmwk_lib} SHARED IMPORTED)
    set_target_properties(${x_fmwk_lib} PROPERTIES IMPORTED_LOCATION ${x_fmwk_path})

    # We want to add include paths for the IMPORTED target (as framework)
    # https://gitlab.kitware.com/cmake/cmake/issues/15689

    # We want to be able to do this from the installed framework without
    # relying on the build tree, but the headers in the framework don't
    # preserve the expected hierarch, i.e., there is some magic in xcode
    # that adds the correct prefix when we do:
    #
    #  #include "drishti/file.h"
    #
    # because that file is really in drishti.framework/Headers/file.h
    
    # Create alias lib for linking: facefilter_renderer -> drishti::drishti
    #add_library(${x_fmwk_lib}::${x_fmwk_lib} ALIAS ${x_fmwk_lib})
    
  else()
    hunter_add_package(drishti)
    find_package(drishti CONFIG REQUIRED)
  endif()
  
endif()

if(NOT DEFINED DRISHTI_HAVE_TO_STRING)
  try_compile(DRISHTI_HAVE_TO_STRING
    "${CMAKE_BINARY_DIR}/try_compile"
    "${CMAKE_CURRENT_LIST_DIR}/cmake/to_string.cpp"
    )
endif()

if(NOT DEFINED DRISHTI_HAVE_LOCALECONV)
  try_compile(DRISHTI_HAVE_LOCALECONV
    "${CMAKE_BINARY_DIR}/try_compile"
    "${CMAKE_CURRENT_LIST_DIR}/cmake/localeconv.cpp"
  )
endif()

################################
### Application dependencies ###
################################

hunter_add_package(OpenCV) # for image IO
find_package(OpenCV REQUIRED)

hunter_add_package(cxxopts) # for CLI
find_package(cxxopts CONFIG REQUIRED)

hunter_add_package(spdlog) # for logging
find_package(spdlog CONFIG REQUIRED)

set(base_deps cxxopts::cxxopts ${OpenCV_LIBS} spdlog::spdlog)
if(TARGET drishti::drishti) # will not be defined if using framework
  list(APPEND base_deps drishti::drishti)
endif()

if(DRISHTI_TEST_BUILD_TESTS)
  hunter_add_package(gauze) # for tests
  find_package(gauze CONFIG REQUIRED)
  list(APPEND base_deps gauze::gauze)
endif()

###############
### Samples ###
###############

#add_subdirectory(eye)
add_subdirectory(facefilter)
