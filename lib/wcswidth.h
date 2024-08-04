#ifndef _WCWIDTH_H
#include <wchar.h>

int wcwidth(wchar_t wc);
int wcswidth(const wchar_t *pwcs, size_t n);

#endif
