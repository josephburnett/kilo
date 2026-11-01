/* What the freestanding link of the mind core (make freestanding) may use
 * besides its own code: the functions Circle provides in circle/util.h, and
 * the libm float functions the planner calls (Circle links the toolchain's
 * libm). They only need to link, never run. And an entry symbol, since there
 * is no crt. */
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

float sinf(float x);
float sinf(float x) { return x; }
float cosf(float x);
float cosf(float x) { return x; }
void sincosf(float x, float *s, float *c);
void sincosf(float x, float *s, float *c)
{
    *s = x;
    *c = x;
}
float tanf(float x);
float tanf(float x) { return x; }
float atanf(float x);
float atanf(float x) { return x; }
float atan2f(float y, float x);
float atan2f(float y, float x) { return y + x; }

/* P3: perception and behaviour compute in double (mind/util/dmath.h). */
double exp(double x);
double exp(double x) { return x; }
double log(double x);
double log(double x) { return x; }
double sin(double x);
double sin(double x) { return x; }
double cos(double x);
double cos(double x) { return x; }
void sincos(double x, double *s, double *c);
void sincos(double x, double *s, double *c)
{
    *s = x;
    *c = x;
}

void kilo_freestanding_entry(void);
void kilo_freestanding_entry(void)
{
    for (;;) {
    }
}
