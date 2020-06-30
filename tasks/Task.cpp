/*-
 * $Copyright$
-*/

#include "tasks/Task.hpp"

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#if !defined(HOSTBUILD)
#include "FreeRTOS.h"
#include "task.h"
#endif /* defined(HOSTBUILD) */

void run_task(void *p_obj) {
    tasks::Task *obj = reinterpret_cast<tasks::Task *>(p_obj);
    obj->run();
#if !defined(HOSTBUILD)
    vTaskSuspend(obj->m_handle);
#endif /* defined(HOSTBUILD) */
}

#if defined(__cplusplus)
} /* extern "C"*/
#endif /* defined(__cplusplus) */

namespace tasks {

/*******************************************************************************
 *
 ******************************************************************************/
Task::Task(const char * const p_name, const unsigned p_priority /* = 0 */, const size_t p_stackSz /* = 0 */) : m_name(p_name), m_handle(nullptr) {
#if !defined(HOSTBUILD)
    xTaskCreate(run_task, p_name, configMINIMAL_STACK_SIZE + p_stackSz, this, tskIDLE_PRIORITY + p_priority, &this->m_handle);
#else
    (void) p_priority;
    (void) p_stackSz;
#endif /* defined(HOSTBUILD) */
}

/*******************************************************************************
 *
 ******************************************************************************/
Task::~Task() {
#if defined(INCLUDE_vTaskDelete) && (INCLUDE_vTaskDelete != 0)
    vTaskDelete(this->m_handle);
#endif
}

} /* namespace tasks */
