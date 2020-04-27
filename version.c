/*-
 * $Copyright$
 */

/*
 * This file generated during the CMake Build Step. The generated file is
 * stored in the Build directory and then used in the build process from there.
 */
#include "build_version.h"

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#if defined(HOSTBUILD)
    const char gSwVersionId[] = BUILD_ID;
    const char gSwBuildTime[] = BUILD_TIME;
#else
    const char gSwVersionId[]  __attribute__((section(".fixeddata"))) = BUILD_ID;
    const char gSwBuildTime[]  __attribute__((section(".fixeddata"))) = BUILD_TIME;
#endif

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

