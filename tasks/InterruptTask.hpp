/*-
 * $Copyright
-*/
#ifndef _INTERRUPT_TASK_HPP_8f63d867_7da4_4413_ac40_0a44a7966039
#define _INTERRUPT_TASK_HPP_8f63d867_7da4_4413_ac40_0a44a7966039

#include <tasks/Task.hpp>

namespace tasks {

class InterruptSource;

class InterruptTask : public Task {
private:
    tasks::InterruptSource * const  m_interruptSource;

    virtual void run(void);

protected:
    virtual void handle_irq(void) = 0;

    void irq_callback(void) const;

public:
    InterruptTask(tasks::InterruptSource *p_interruptSource,
                             const char * const p_name,
                             const unsigned p_priority = 0,
                             const size_t p_stackSz = 0);
    virtual ~InterruptTask();
};

} /* namespace tasks */

#endif /* _INTERRUPT_TASK_HPP_8f63d867_7da4_4413_ac40_0a44a7966039 */
