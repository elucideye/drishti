set(top_dir "${CMAKE_CURRENT_LIST_DIR}/../..")

if(FACEFILTER_ANDROID_STUDIO)
  set(gen_toolchain "${CMAKE_CURRENT_BINARY_DIR}/generated/toolchain.cmake")
  configure_file(
    "${top_dir}/cmake/template/toolchain.cmake.in"
    "${gen_toolchain}"
    @ONLY
    )
  set(CMAKE_TOOLCHAIN_FILE "${gen_toolchain}" CACHE PATH "" FORCE)
endif()

# By default, the source code for all managed dependencies will be removed after
# building and installing to the cache.  This behavior is consistent with most
# installed libraries (i.e., /usr/local/lib/*), but it does prevent stepping
# through the dependencies in a debugger in cases where a problem is not
# contained within the drishti project sources.  In such cases, you can set
# HUNTER_KEEP_PACKAGE_SOURCES=ON from the command line during the project
# configuration and the source will be left for all packages when they are
# created.  This setting must be used before a package is installed -- it
# won't be applied retroactively to installed packages.  In order to re-install
# a package with sources you can always remove the cache
# (i.e., rm -rf ${HOME}/.hunter) or, less drastically you can modify the
# CONFIG-ID of an installed package to trigger the configuration and
# installation steps.  This can be done by modifying the input CMAKE_ARGS
# list in a hunter_config() call.  In the following example KEEP_SOURCES=1
# is  added to trigger a re-installation:
#
#   hunter_config(foo VERSION ${HUNTER_foo_VERSION} CMAKE_ARGS KEEP_SOURCES=1)
#
# The HUNTER_KEEP_PACKAGE_SOURCES development feature is described here:
#
# In order to support stepping through package sources you will also have to
# make sure that debug versions of the packages are installed.  This will
# happen by default, but will not happen if you specify a release only build
# using HUNTER_CONFIGURATION_TYPES=Release
# https://docs.hunter.sh/en/latest/reference/user-variables.html#hunter-keep-package-sources
option(HUNTER_KEEP_PACKAGE_SOURCES "Keep installed package sources for debugging (caveat...)" ON)

include("${top_dir}/cmake/HunterGate.cmake")

if(HUNTER_PACKAGE_BUILD)
  # URL/SHA1 will not be used actually, settings will be inherited
  HunterGate(URL "${HUNTER_URL}" SHA1 "${HUNTER_SHA1}")
else()
  set(drishti_upload_init "${top_dir}/drishti-upload/init.cmake")

  if(NOT EXISTS "${drishti_upload_init}")
    message(
      FATAL_ERROR
      "File does not exist:\n"
      "  ${drishti_upload_init}\n"
      "Run this command if submodule is not initialized:\n"
      "  git submodule update --init --recursive ."
    )
  endif()

  # CI: Locked CI build configuration
  # HunterGate called here
  include("${drishti_upload_init}")
endif()

# Used in drishti-upload/config.cmake {

# DRISHTI_BUILD_MIN_SIZE:
#   This can alter the hunter-package options for
#   various dependencies in order to produce smaller builds,
#   such as eliminating code related to model training.
#
option(DRISHTI_BUILD_MIN_SIZE "Build minimum size lib (exclude training)" ON)

option(DRISHTI_BUILD_OGLES_GPGPU "Build with OGLES_GPGPU" ON)
option(DRISHTI_OPENGL_ES3 "Support OpenGL ES 3.0 (default 2.0)" OFF)
option(DRISHTI_SERIALIZE_WITH_CVMATIO "Perform serialization with cvmatio" OFF)

# }
