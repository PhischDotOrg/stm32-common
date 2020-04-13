/*-
 * $Copyright$
-*/


#if defined(__cplusplus)
extern "C" {
#endif /* defined (__cplusplus) */

void
__cxa_pure_virtual(void) {
    while (1) ;
};

/*
 * These are needed for static variables declared in C++ methods. This solution
 * was found at
 *
 *   https://answers.launchpad.net/gcc-arm-embedded/+question/221105
 *
 * (on July 15th, 2014)
 */
__extension__ typedef int __guard __attribute__((mode (__DI__)));
int __cxa_guard_acquire(__guard *g) {
    return !*(char *)(g);
};

void __cxa_guard_release (__guard *g) {
    *(char *)g = 1;
};

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined (__cplusplus) */

void
operator delete(void * /* p_obj */) throw() {
    while (1) ;
}

void
operator delete(void * /* p_obj */, unsigned int /* p_size */) throw() {
    while (1) ;
}
