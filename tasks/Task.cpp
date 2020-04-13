/*-
 * $Copyright$
-*/

#include "tasks/Task.hpp"

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#include "FreeRTOS.h"
#include "task.h"

void run_task(void *p_obj) {
    tasks::Task *obj = reinterpret_cast<tasks::Task *>(p_obj);
    obj->run();
    vTaskSuspend(obj->m_handle);
}

#if defined(__cplusplus)
} /* extern "C"*/
#endif /* defined(__cplusplus) */

namespace tasks {

/*******************************************************************************
 *
 ******************************************************************************/
Task::Task(const char * const p_name, const unsigned p_priority /* = 0 */, const size_t p_stackSz /* = 0 */) : m_name(p_name), m_handle(NULL) {
    xTaskCreate(run_task, p_name, configMINIMAL_STACK_SIZE + p_stackSz, this, tskIDLE_PRIORITY + p_priority, &this->m_handle);
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
