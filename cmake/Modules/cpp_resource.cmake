include(CMakeParseArguments)

function(find_resource_compiler)

  if(NOT (COMMAND cpp_rsc))
    include(ExternalProject)
    ExternalProject_Add(cpp_rsc_prj
      GIT_REPOSITORY git@github.com:orex/cpp_rsc.git
      GIT_TAG master
      BUILD_COMMAND ""
      CMAKE_ARGS
      "-Gdefault @CMAKE_GENERATOR@""
      INSTALL_COMMAND ""
      
      )
    
    get_property(RSC_BIN_FOLDER TARGET cpp_rsc_prj PROPERTY _EP_BINARY_DIR)
    set_property(GLOBAL PROPERTY CPPRSC_CMD ${RSC_BIN_FOLDER}/src/${CMAKE_CFG_INTDIR}/cpp_rsc)
  else() 
    set_property(GLOBAL PROPERTY CPPRSC_CMD cpp_rsc)
  endif()
  
endfunction(find_resource_compiler)                        

function(add_resource name)

  set(oneValueArgs OUTPUT RC_WORK_DIR SUFFIX_HEAD SUFFIX_SRC NAMESPACE DATA_WIDTH)
  cmake_parse_arguments(ADR "" "${oneValueArgs}" "" ${ARGN} )

  if("${ADR_OUTPUT}" STREQUAL "")
    set(ADR_OUTPUT ${name})
  endif()
  
  if("${ADR_SUFFIX_HEAD}" STREQUAL "")
    set(ADR_SUFFIX_HEAD "h")
  endif()
  
  if("${ADR_SUFFIX_SRC}" STREQUAL "")
    set(ADR_SUFFIX_SRC "cpp")
  endif()
  
  set(RSC_FILE_NAME ${CMAKE_CURRENT_BINARY_DIR}/${name}.rsc)
  set(RSC_OUT_H ${CMAKE_CURRENT_BINARY_DIR}/${ADR_OUTPUT}.${ADR_SUFFIX_HEAD})
  set(RSC_OUT_CPP ${CMAKE_CURRENT_BINARY_DIR}/${ADR_OUTPUT}.${ADR_SUFFIX_SRC})

  file(WRITE  ${RSC_FILE_NAME} "[general]\n")
  file(APPEND ${RSC_FILE_NAME} "output-file-name=${ADR_OUTPUT}\n")
  
  if(NOT ("${ADR_RC_WORK_DIR}" STREQUAL ""))
    file(APPEND ${RSC_FILE_NAME} "base-path=${ADR_RC_WORK_DIR}\n")  
  endif()  
  
  file(APPEND ${RSC_FILE_NAME} "suffix-header=${ADR_SUFFIX_HEAD}\n")
  file(APPEND ${RSC_FILE_NAME} "suffix-src=${ADR_SUFFIX_SRC}\n")
  
  if(NOT ("${ADR_NAMESPACE}" STREQUAL ""))
    file(APPEND ${RSC_FILE_NAME} "namespace=${ADR_NAMESPACE}\n")
  endif()
  
  if(NOT ("${ADR_DATA_WIDTH}" STREQUAL ""))
    file(APPEND ${RSC_FILE_NAME} "data-width=${ADR_DATA_WIDTH}\n\n")
  endif()  

  add_custom_target(${name}
    DEPENDS ${RSC_OUT_H} ${RSC_OUT_CPP})
  
  set_property(TARGET ${name} PROPERTY _AR_RSC_FILE ${RSC_FILE_NAME})
  set_property(TARGET ${name} PROPERTY _AR_H_FILE   ${RSC_OUT_H})
  set_property(TARGET ${name} PROPERTY _AR_SRC_FILE ${RSC_OUT_CPP})
  set_property(TARGET ${name} PROPERTY _AR_H_DIR ${CMAKE_CURRENT_BINARY_DIR})  

  get_property(CMDRSC GLOBAL PROPERTY CPPRSC_CMD)
  if("${CMDRSC}" STREQUAL "")
    unset(CMDRSC)
    set(CMDRSC "cpp_rsc")
  endif()
  
  if(TARGET cpp_rsc_prj)
    add_dependencies(${name} cpp_rsc_prj)
  endif()
  
  add_custom_command(OUTPUT ${RSC_OUT_H} ${RSC_OUT_CPP}
    COMMAND ${CMDRSC} ${RSC_FILE_NAME}
    DEPENDS ${RSC_FILE_NAME})
  
endfunction(add_resource name)

function(link_resource_file name)

  set(options TEXT)
  set(oneValueArgs FILE VARIABLE)
  cmake_parse_arguments(ARL "${options}" "${oneValueArgs}" "" ${ARGN} )
  
  get_property(RSC_FILE_NAME TARGET ${name} PROPERTY _AR_RSC_FILE)
  
  file(APPEND ${RSC_FILE_NAME} "[file]\n")
  file(APPEND ${RSC_FILE_NAME} "file-path=${ARL_FILE}\n")
  file(APPEND ${RSC_FILE_NAME} "var-name=${ARL_VARIABLE}\n")  
  file(APPEND ${RSC_FILE_NAME} "text-file=${ARL_TEXT}\n\n")
  
  set_source_files_properties(${RSC_FILE_NAME} PROPERTIES OBJECT_DEPENDS ${ARL_FILE})  

endfunction(link_resource_file name)
