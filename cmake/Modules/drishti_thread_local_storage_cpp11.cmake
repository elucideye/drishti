function(drishti_thread_local_storage_cpp11 varName)
  include(CheckCSourceCompiles)
  check_cxx_source_compiles("static thread_local int tls; int main(void) { return 0; }"
    HAVE_THREAD_LOCAL_STORAGE_CPP11
    )
  set(${varName} ${HAVE_THREAD_LOCAL_STORAGE_CPP11} PARENT_SCOPE)
endfunction(drishti_thread_local_storage_cpp11)
