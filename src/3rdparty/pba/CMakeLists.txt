cmake_minimum_required(VERSION 3.0)

include("cmake/HunterGate.cmake")
HunterGate(
  URL "https://github.com/ruslo/hunter/archive/v0.18.18.tar.gz"
  SHA1 "ae1f72b9e8779a58f763179c77f609c0587d26cd"
  )

project(boost-pba VERSION 1.0.0)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

option(BOOST_PBA_BUILD_EXAMPLES "Build examples" OFF)
option(BOOST_PBA_INSTALL "Perform installation" ON)

set(BOOST_PBA_3RDPARTY_LIBS "")

#############
### boost ###
#############

hunter_add_package(Boost COMPONENTS filesystem system serialization)
find_package(Boost CONFIG REQUIRED filesystem system serialization)
include_directories(${Boost_INCLUDE_DIRS})
list(APPEND BOOST_PBA_3RDPARTY_LIBS
  Boost::filesystem
  Boost::system
  Boost::serialization
  )

###########
### pba ###
###########

set(boost_pba_srcs
  portable_binary_archive.hpp
  portable_binary_iarchive.cpp
  portable_binary_iarchive.hpp
  portable_binary_oarchive.cpp
  portable_binary_oarchive.hpp
  )

add_library(boost-pba ${boost_pba_srcs})
target_link_libraries(boost-pba PUBLIC ${BOOST_PBA_3RDPARTY_LIBS})
target_include_directories(boost-pba PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../>"
  "$<INSTALL_INTERFACE:include>"
  )

################
### Examples ###
################

# TODO

###############
### Install ###
###############

if(NOT BOOST_PBA_INSTALL)
  return()
endif()

### Install ###
set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")

#set(config_install_dir "lib/cmake/${PROJECT_NAME}")
set(config_install_dir "lib/cmake/pba")
set(include_install_dir "include")

set(version_config "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake")
set(project_config "${generated_dir}/${PROJECT_NAME}Config.cmake")
set(targets_export_name "${PROJECT_NAME}Targets")
set(namespace "${PROJECT_NAME}::")

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${version_config}" COMPATIBILITY SameMajorVersion
)

# Note: use 'targets_export_name'
configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/Config.cmake.in"
    "${project_config}"
     INSTALL_DESTINATION "${config_install_dir}"
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT "${targets_export_name}"
    LIBRARY DESTINATION "lib"
    ARCHIVE DESTINATION "lib"
    RUNTIME DESTINATION "bin"
    INCLUDES DESTINATION "${include_install_dir}"
)

install(
    FILES
    portable_binary_archive.hpp
    portable_binary_iarchive.hpp
    portable_binary_oarchive.hpp
    DESTINATION "${include_install_dir}/pba"
)

install(
  FILES "${project_config}" "${version_config}"
  DESTINATION "${config_install_dir}"
  )

install(
  EXPORT "${targets_export_name}"
  NAMESPACE "${namespace}"
  DESTINATION "${config_install_dir}"
  )
