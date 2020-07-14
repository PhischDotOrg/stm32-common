/*-
 * $Copyright$
 */

/*******************************************************************************
 *
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

char stext, etext;
char sdata, edata;
char sbss, ebss;
char bstack, estack;

extern const char gFixedDataBegin = 0;
extern const char gFixedDataUsed = 0;
extern const char gFixedDataEnd = 0;


void
taskDISABLE_INTERRUPTS(void) {

}

long
xPortStartScheduler( void ) {
    return (0);
}

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */
