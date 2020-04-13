/*-
 * $Copyright$
-*/

#ifndef _MP3_DECODE_H_5cc767c8_c3cb_4fb0_b5cd_433c01152b40
#define _MP3_DECODE_H_5cc767c8_c3cb_4fb0_b5cd_433c01152b40

#include <sys/types.h>
#include <stdint.h>

#if defined(__cplusplus)
extern "C"{
#endif /* defined(__cplusplus) */
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"

#include "mp3dec.h"
#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

#include <uart/UartDevice.hpp>

typedef struct buffer_s {
    uint8_t *   m_base;
    size_t      m_length;
    off_t       m_pos;
} buffer_t;

typedef struct mp3_decode_params_s {
    xQueueHandle *      m_pcm_queue;
    uart::UartDevice *  m_uart;
} mp3_decode_params_t;

#if defined(__cplusplus)
extern "C"{
#endif /* defined(__cplusplus) */
void mp3_decode(void * p_param);

extern HMP3Decoder decoder;
extern buffer_t mp3buf;

#if defined(__cplusplus)
} /* extern "C"*/
#endif /* defined(__cplusplus) */

#endif /* _MP3_DECODE_H_5cc767c8_c3cb_4fb0_b5cd_433c01152b40 */
