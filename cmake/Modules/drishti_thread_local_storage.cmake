function(drishti_thread_local_storage varName)
  include(CheckCSourceCompiles)
  check_c_source_compiles("
#if defined (__GNUC__)
    #define ATTRIBUTE_TLS __thread
#elif defined (_MSC_VER)
    #define ATTRIBUTE_TLS __declspec(thread)
#else // !__GNUC__ && !_MSC_VER
    #error \"Define a thread local storage qualifier for your compiler/platform!\"
#endif
ATTRIBUTE_TLS int tls;
int main(void) { return 0;
}" HAVE_THREAD_LOCAL_STORAGE)
  set(${varName} ${HAVE_THREAD_LOCAL_STORAGE} PARENT_SCOPE)
endfunction(drishti_thread_local_storage)