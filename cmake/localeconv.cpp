/* localeconv example */
#include <locale.h>     /* setlocale, LC_MONETARY, struct lconv, localeconv */

int main ()
{
    struct lconv * lc;
    lc=localeconv();
    return 0;
}
