# Top level folder for QT applications:
add_subdirectory(qt)

if(DRISHTI_USE_TEXT_ARCHIVES)
  if(NOT ANDROID AND NOT IOS)
    add_subdirectory(pba)
  endif()
endif()

###########################
### Command line parser ###
###########################

option(DRISHTI_HAS_CXXOPTS "Has cxxopts" ON)
option(DRISHTI_USE_LOCAL_CXXOPTS "Use local cxxopts" ON)

if(DRISHTI_HAS_CXXOPTS)
  if(DRISHTI_USE_LOCAL_CXXOPTS)
    option(CXXOPTS_BUILD_EXAMPLES "Set to ON to build examples" OFF)
    option(CXXOPTS_BUILD_TESTS "Set to ON to build tests" OFF)
    option(CXXOPTS_INSTALL "Install package." OFF)    
    add_subdirectory("${CMAKE_SOURCE_DIR}/src/3rdparty/cxxopts" "${CMAKE_CURRENT_BINARY_DIR}/3rdparty/cxxopts")
    add_library(cxxopts::cxxopts ALIAS cxxopts)
  else()
    hunter_add_package(cxxopts)
    find_package(cxxopts CONFIG REQUIRED)
  endif()
endif()

# std::regex >= gcc 4.8
if(DRISHTI_HAS_CXXOPTS)
  add_subdirectory(eye)
endif()
