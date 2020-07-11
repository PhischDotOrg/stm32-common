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

#if !defined(HOSTBUILD)
    #define FIXED_DATA_SECTION __attribute__((section(".fixeddata")))
#else
    #define FIXED_DATA_SECTION
#endif

const char gProjectName[] FIXED_DATA_SECTION = PROJECT_NAME;
const char gSwVersionId[] FIXED_DATA_SECTION = BUILD_ID;
const char gSwBuildTime[] FIXED_DATA_SECTION = BUILD_TIME;

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

