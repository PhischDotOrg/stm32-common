/*-
 * $Copyright$
-*/

#ifndef _PERIODIC_TASK_HPP_C4A4891A_F2B3_413B_A08F_D0C34AC00BD3
#define _PERIODIC_TASK_HPP_C4A4891A_F2B3_413B_A08F_D0C34AC00BD3

#include <tasks/Task.hpp>
#include <cstdint>
#include <phisch/log.h>

namespace tasks {

class PeriodicTask : public Task {
private:
#if defined(HOSTBUILD)
    typedef unsigned TickType_t;

    #define portTICK_PERIOD_MS 1
#endif /* defined(HOSTBUILD) */

    const TickType_t    m_period;

public:
    PeriodicTask(const char * const p_name, const unsigned p_priority, const unsigned p_periodMs)
       : Task(p_name, p_priority), m_period(p_periodMs / portTICK_PERIOD_MS) {

    };

    virtual ~PeriodicTask() override {

    };

    virtual void executePeriod(void) = 0;

public:
    virtual void run(void) override {
        PHISCH_LOG("Task '%s' ticking at %lu ms\r\n", this->m_name, this->m_period * portTICK_PERIOD_MS);

        do {
            executePeriod();

#if !defined(HOSTBUILD)
            vTaskDelay(m_period);
#endif /* defined(HOSTBUILD) */
        } while (1);
    };
};

}; /* namespace tasks */

#endif /* _PERIODIC_TASK_HPP_C4A4891A_F2B3_413B_A08F_D0C34AC00BD3 */
