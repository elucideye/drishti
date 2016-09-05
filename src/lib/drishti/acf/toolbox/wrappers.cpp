#include "drishti/acf/toolbox/wrappers.hpp"

// platform independent aligned memory allocation (see also alFree)
void* alMalloc( size_t size, int alignment )
{
    const size_t pSize = sizeof(void*), a = alignment-1;
    void *raw = wrMalloc(size + a + pSize);
    void *aligned = (void*) (((size_t) raw + pSize + a) & ~a);
    *(void**) ((size_t) aligned-pSize) = raw;
    return aligned;
}

// platform independent alignned memory de-allocation (see also alMalloc)
void alFree(void* aligned)
{
    void* raw = *(void**)((char*)aligned-sizeof(void*));
    wrFree(raw);
}
