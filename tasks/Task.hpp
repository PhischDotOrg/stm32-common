/*-
 * $Copyright$
-*/

#ifndef _TASK_HPP_e3c507ff_6ad2_4a1f_a8cc_6dea51135a50
#define _TASK_HPP_e3c507ff_6ad2_4a1f_a8cc_6dea51135a50

#if defined(__cplusplus)
extern "C"{
#endif /* defined(__cplusplus) */
void run_task(void *);

// #if !defined(HOSTBUILD)
#include <FreeRTOS.h>
#include <task.h>
// #else
// typedef void * TaskHandle_t;
// #endif

#if defined(__cplusplus)
} /* extern "C"*/
#endif /* defined(__cplusplus) */

#include <sys/types.h>

namespace tasks {

class Task {
    friend void ::run_task(void *);
protected:
    const char * const m_name;
    TaskHandle_t m_handle;

    virtual void run(void) = 0;

public:
    Task(const char * const p_name, const unsigned p_priority = 0, const size_t p_stackSz = 0);
    virtual ~Task();
}; /* class Task */

} /* namespace tasks */

#endif /* _TASK_HPP_e3c507ff_6ad2_4a1f_a8cc_6dea51135a50 */
