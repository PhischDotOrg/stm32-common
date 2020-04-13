/*-
 * $Copyright$
-*/

#ifndef _INTERRUPT_SOURCE_HPP_560c35cf_bbfb_468f_901b_20f28cfdc066
#define _INTERRUPT_SOURCE_HPP_560c35cf_bbfb_468f_901b_20f28cfdc066

namespace tasks {

class InterruptSource {
public:
    virtual int enable(void) = 0;
    virtual int disable(void) = 0;
    virtual int registerHandler(const InterruptTask * const p_interruptHandler) = 0;
    virtual int deregisterHandler(const InterruptTask * const p_interruptHandler) = 0;
};

}
#endif /* _INTERRUPT_SOURCE_HPP_560c35cf_bbfb_468f_901b_20f28cfdc066 */
