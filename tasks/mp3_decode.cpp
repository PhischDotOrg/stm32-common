/*-
 * $Copyright$
-*/

#include "mp3_decode.hpp"

#include "mp3dec.h"

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/queue.h"

#include <sys/types.h>
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <strings.h>

#include "pcm_play.hpp"

#if defined(MP3_DEBUG)
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#endif /* defined(MP3_DEBUG) */

#include <uart/UartDevice.hpp>
#include <uart/UartDevice.cpp>

#define PCM_BUF_SZ 4096 * 2

static uint16_t pcmcur;
static uint16_t pcmbuf[PCM_QUEUE_LEN][PCM_BUF_SZ];

#if defined(__cplusplus)
extern "C"{
#endif /* defined(__cplusplus) */

HMP3Decoder decoder;
buffer_t mp3buf;

#if defined(MP3_DEBUG)
static const char * const   mp3debug_template = "mp3_decode.XXXXXX";
#endif /* defined(MP3_DEBUG) */

void
mp3_decode(void * p_param) {
    MP3FrameInfo frameInfo;
    int offs, rc, bytesLeft;
    mp3_decode_params_t *params = (mp3_decode_params_t *) p_param;
    xQueueHandle pcm_queue = *params->m_pcm_queue;
    uart::UartDevice *uart = params->m_uart;
    pcm_msg_t msg;
    void *inbuf;

    decoder = MP3InitDecoder();

#if defined(MP3_DEBUG)
    char *mp3debug_filename = strdup(mp3debug_template);
    int mp3debug_fd = mkstemp(mp3debug_filename);
    assert(mp3debug_fd > 0);
#endif /* defined(MP3_DEBUG) */

    do {
        offs = MP3FindSyncWord(mp3buf.m_base + mp3buf.m_pos, mp3buf.m_length - mp3buf.m_pos);
//         assert(offs >= 0);

        mp3buf.m_pos += offs;

        rc = MP3GetNextFrameInfo(decoder, &frameInfo, mp3buf.m_base + mp3buf.m_pos);
        if (rc < 0) {
            uart->printf("Failed to get info for frame at offset %i\n", (int) mp3buf.m_pos);
            mp3buf.m_pos++;
            continue;
        }

        msg.m_base = pcmbuf[pcmcur];
        bzero(pcmbuf[pcmcur], PCM_BUF_SZ);
        pcmcur = (pcmcur + 1) % PCM_QUEUE_LEN;

        inbuf = mp3buf.m_base + mp3buf.m_pos;
        bytesLeft = mp3buf.m_length - mp3buf.m_pos;

        rc = MP3Decode(decoder, (unsigned char **) &inbuf, &bytesLeft, (short *) msg.m_base, 0);

        mp3buf.m_pos += (mp3buf.m_length - mp3buf.m_pos) - bytesLeft;

        if (rc) {
            uart->printf("MP3Decode() returned rc=%i\n", rc);
            continue;
        }

        MP3GetLastFrameInfo(decoder, &frameInfo);

        uart->printf("[0x%.8x] Chans=%i, Sample Rate=%i, Samples=%i, Bit Rate=%i, Bits per Sample=%i\n",
          (int) mp3buf.m_pos, frameInfo.nChans, frameInfo.samprate, frameInfo.outputSamps,
          frameInfo.bitrate, frameInfo.bitsPerSample);

        msg.m_channels = frameInfo.nChans;
        msg.m_rate = frameInfo.samprate;
        msg.m_length = (frameInfo.bitsPerSample >> 3) * frameInfo.nChans * frameInfo.outputSamps;

#if defined(MP3_DEBUG)
        write(mp3debug_fd, msg.m_base, msg.m_length);
#endif /* defined(MP3_DEBUG) */

        xQueueSend(pcm_queue, &msg, portTICK_RATE_MS * 5);
    } while ((unsigned) mp3buf.m_pos < mp3buf.m_length);

#if defined(MP3_DEBUG)
    free(mp3debug_filename);
    close(mp3debug_fd);
#endif /* defined(MP3_DEBUG) */

    MP3FreeDecoder(decoder);

    while (1) { };
}
#if defined(__cplusplus)
} /* extern "C"*/
#endif /* defined(__cplusplus) */
