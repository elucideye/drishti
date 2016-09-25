function(drishti_strip drishti_library)
  # Xcode
  set_target_properties(${drishti_library}
    PROPERTIES
    XCODE_ATTRIBUTE_COPY_PHASE_STRIP "YES"
    XCODE_ATTRIBUTE_STRIP_INSTALLED_PRODUCT "YES"
    XCODE_ATTRIBUTE_STRIP_STYLE "non-global"
    XCODE_ATTRIBUTE_STRIPFLAGS "-x -u -r"
    XCODE_ATTRIBUTE_DEAD_CODE_STRIPPING "YES"
    XCODE_ATTRIBUTE_DEPLOYMENT_POSTPROCESSING "YES"
    XCODE_ATTRIBUTE_GENERATE_MASTER_OBJECT_FILE "YES" # "Perform Single-Object Prelink"
    )
endfunction(drishti_strip)
