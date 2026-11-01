/* What the freestanding link of the mind core (make freestanding) may use
 * besides its own code: the functions Circle provides in circle/util.h. They
 * only need to link, never run. And an entry symbol, since there is no crt. */
#include <stddef.h>

void *memcpy(void *dst, const void *src, size_t n)
{
    (void)src;
    (void)n;
    return dst;
}

void *memset(void *dst, int c, size_t n)
{
    (void)c;
    (void)n;
    return dst;
}

int memcmp(const void *a, const void *b, size_t n)
{
    (void)a;
    (void)b;
    (void)n;
    return 0;
}

void kilo_freestanding_entry(void);
void kilo_freestanding_entry(void)
{
    for (;;) {
    }
}
