/*-
 * $Copyright$
-*/

#include <tasks/InterruptTask.hpp>
#include <tasks/InterruptSource.hpp>

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"

#if defined(__cplusplus)
} /* extern "C"*/
#endif /* defined(__cplusplus) */

namespace tasks {

/*******************************************************************************
 *
 ******************************************************************************/
InterruptTask::InterruptTask(tasks::InterruptSource *p_interruptSource,
                             const char * const p_name,
                             const unsigned p_priority /* = 0 */,
                             const size_t p_stackSz /* = 0 */)
  : tasks::Task(p_name, p_priority, p_stackSz), m_interruptSource(p_interruptSource) {
    m_interruptSource->registerHandler(this);
}

/*******************************************************************************
 *
 ******************************************************************************/
InterruptTask::~InterruptTask() {
    m_interruptSource->deregisterHandler(this);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InterruptTask::run(void) {
    do {
        vTaskSuspend(NULL);
        this->handle_irq();
    } while (1);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InterruptTask::irq_callback(void) const {
    xTaskResumeFromISR(this->m_handle);
}

} /* namespace tasks */
