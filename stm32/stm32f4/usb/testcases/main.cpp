/*-
 * $Copyright$
-*/

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <stdio.h>

#if 0
int
main(int argc, char **argv) {
    ::testing::InitGoogleMock(&argc, argv);

    return RUN_ALL_TESTS();
}
#endif

/******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* defined (__cplusplus) */
/******************************************************************************/

void
debug_printf(const char * const p_fmt, ...) {
    va_list va;
    va_start(va, p_fmt);

    vprintf(p_fmt, va);

    va_end(va);
}

/******************************************************************************/
#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined (__cplusplus) */
/******************************************************************************/

