/*-
 * $Copyright$
-*/

#ifndef _PCM_PLAY_H_df698c64_734b_4fc8_9c17_452174cc7f68
#define _PCM_PLAY_H_df698c64_734b_4fc8_9c17_452174cc7f68

#if defined(__cplusplus)
extern "C"{
#endif /* defined(__cplusplus) */
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"

#include "mp3dec.h"
#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

#include <sys/types.h>
#include <stdint.h>

#if defined(__cplusplus)
extern "C"{
#endif /* defined(__cplusplus) */
void pcm_play(void * p_param);
#if defined(__cplusplus)
} /* extern "C"*/
#endif /* defined(__cplusplus) */

#include <pcm/PcmAccess.hpp>
#include <uart/UartDevice.hpp>

typedef struct pcm_msg_s {
    void *      m_base;
    size_t      m_length;
    uint16_t    m_rate;
    uint8_t     m_channels;
} pcm_msg_t;

typedef struct pcm_play_params_s {
    xQueueHandle *      m_pcm_queue;
    pcm::PcmAccess *    m_pcm;
    uart::UartDevice *  m_uart;
} pcm_play_params_t;

#define PCM_QUEUE_LEN 4

#endif /* _PCM_PLAY_H_df698c64_734b_4fc8_9c17_452174cc7f68 */
