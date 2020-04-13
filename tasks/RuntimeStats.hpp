/*-
 * $Copyright$
-*/

#ifndef _RUNTIME_STATS_HPP_1936f317_8980_48d4_9358_20174a21235c
#define _RUNTIME_STATS_HPP_1936f317_8980_48d4_9358_20174a21235c

#include <tasks/Task.hpp>

namespace tasks {

template<typename UartT>
class RuntimeStatsT : public Task {
private:
    UartT &         m_uart;
    const unsigned  m_periodInMs;
    char            m_buffer[512];

    virtual void    run(void);

public:
    RuntimeStatsT(const char * const p_name, const unsigned p_priority,
      UartT &p_uart, const unsigned p_periodInMs);
    virtual ~RuntimeStatsT();
};

}; /* namespace tasks */

#include "RuntimeStats.cpp"

#endif /* _RUNTIME_STATS_HPP_1936f317_8980_48d4_9358_20174a21235c */