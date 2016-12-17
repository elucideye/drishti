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
    # Note: Using this option in combination with thread_local and shared libraries crashes
    # the linker step with the following error:
    #
    # ld: Assertion failed: (0 && "need to handle arm64 -r reloc"), function encodeSectionReloc,
    # file /Library/Caches/com.apple.xbs/Sources/ld64/ld64-264.3.102/src/ld/LinkEditClassic.hpp, line 1907.
    #
    # This should can be enabled optionally for modules that do not require thread_local.
    XCODE_ATTRIBUTE_GENERATE_MASTER_OBJECT_FILE "YES" # "Perform Single-Object Prelink"
    )
endfunction(drishti_strip)
