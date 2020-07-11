/*-
 * $Copyright$
 */

#ifndef __COMMON_INFRASTRUCTURE_HPP_5F719FD2_3253_4E80_B9BD_B6F64D4A6D50
#define __COMMON_INFRASTRUCTURE_HPP_5F719FD2_3253_4E80_B9BD_B6F64D4A6D50

#if defined(__cplusplus)
extern "C" {
#endif  /* defined(__cplusplus) */

void PrintStartupMessage(unsigned p_sysclk, unsigned p_ahb, unsigned p_apb1, unsigned p_apb2);

extern char stext, etext;
extern char sdata, edata;
extern char sbss, ebss;
extern char bstack, estack;

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

#endif /* __COMMON_INFRASTRUCTURE_HPP_5F719FD2_3253_4E80_B9BD_B6F64D4A6D50 */