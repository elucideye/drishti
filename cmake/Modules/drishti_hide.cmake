function(drishti_hide drishti_library)
  # Xcode
  set_target_properties(${drishti_library}
    PROPERTIES
    CXX_VISIBILITY_PRESET hidden  ### HIDE
    VISIBILITY_INLINES_HIDDEN "ON"
    XCODE_ATTRIBUTE_GCC_INLINES_ARE_PRIVATE_EXTERN "YES"
    XCODE_ATTRIBUTE_GCC_SYMBOLS_PRIVATE_EXTERN "YES"
    )
endfunction(drishti_hide)
