/*-
 * $Copyright$
-*/

#include "pcm_play.hpp"

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"

#include <pcm/PcmAccess.hpp>

#include <sys/time.h>
#include <stdlib.h>
#include <assert.h>

#if defined(__cplusplus)
extern "C"{
#endif /* defined(__cplusplus) */
void
pcm_play(void * p_params) {
    pcm_play_params_t *params = reinterpret_cast<pcm_play_params_t *>(p_params);
    xQueueHandle pcm_queue = params->m_pcm_queue;
    pcm::PcmAccess *pcm = params->m_pcm;
    pcm_msg_t msg;

    pcm->setup(2, 44100);

    do {
        xQueueReceive(pcm_queue, &msg, 100 / portTICK_RATE_MS);

        pcm->write((uint8_t *) msg.m_base, msg.m_length);
    } while(1);
}
#if defined(__cplusplus)
} /* extern "C"*/
#endif /* defined(__cplusplus) */
